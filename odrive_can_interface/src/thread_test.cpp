// Thread Test Cases for OdriveCANSystem
// Build: colcon build --packages-select odrive_can_interface
// Run: ros2 run odrive_can_interface thread_test

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <vector>
#include <iostream>
#include <iomanip>

#include "odrive_can_interface/shared_memory.hpp"

using namespace std::chrono_literals;
using namespace odrive_can_interface;

class ThreadTestNode : public rclcpp::Node
{
public:
    ThreadTestNode() : Node("thread_test")
    {
        RCLCPP_INFO(get_logger(), "=== Thread Test Cases ===");
        
        // Parameters
        test_duration_s_ = declare_parameter<double>("duration_s", 5.0);
        
        // Open SHM
        if (!shmitf_.open(false))
        {
            RCLCPP_ERROR(get_logger(), "Failed to open shared memory");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Running tests for %.1f seconds...", test_duration_s_);
        
        // Run all tests
        test_shm_race_condition();
        test_timing_jitter();
        test_sequence_continuity();
        test_data_consistency();
        
        RCLCPP_INFO(get_logger(), "=== All Tests Complete ===");
    }

private:
    SharedMemoryInterface shmitf_;
    double test_duration_s_{5.0};
    
    // ========== TEST 1: SHM Race Condition ==========
    void test_shm_race_condition()
    {
        RCLCPP_INFO(get_logger(), "\n--- TEST 1: SHM Race Condition ---");
        
        std::atomic<uint64_t> read_count{0};
        std::atomic<uint64_t> torn_read_count{0};
        std::atomic<bool> running{true};
        
        // Writer thread (simulates HWI writing state)
        std::thread writer([&]() {
            uint32_t seq = 0;
            while (running)
            {
                if (auto *state = shmitf_.state())
                {
                    // Write sequence first, then data, then sequence again
                    state->sequence_id = seq;
                    for (size_t i = 0; i < state->axis_count && i < SHM_MAX_AXES; ++i)
                    {
                        state->axes[i].position = static_cast<float>(seq);
                        state->axes[i].velocity = static_cast<float>(seq);
                    }
                    seq++;
                }
                std::this_thread::sleep_for(10ms); // 100Hz
            }
        });
        
        // Reader thread (simulates HSH reading state)
        std::thread reader([&]() {
            while (running)
            {
                if (auto *state = shmitf_.state())
                {
                    uint32_t seq_before = state->sequence_id;
                    float pos = state->axes[0].position;
                    float vel = state->axes[0].velocity;
                    uint32_t seq_after = state->sequence_id;
                    
                    read_count++;
                    
                    // Check for torn read: seq changed during read
                    if (seq_before != seq_after)
                    {
                        torn_read_count++;
                    }
                    // Check for inconsistent data
                    if (static_cast<uint32_t>(pos) != static_cast<uint32_t>(vel))
                    {
                        torn_read_count++;
                    }
                }
                std::this_thread::sleep_for(5ms); // 200Hz
            }
        });
        
        std::this_thread::sleep_for(std::chrono::duration<double>(test_duration_s_));
        running = false;
        
        writer.join();
        reader.join();
        
        double torn_rate = 100.0 * torn_read_count / std::max(1UL, read_count.load());
        RCLCPP_INFO(get_logger(), "  Reads: %lu, Torn reads: %lu (%.2f%%)",
                    read_count.load(), torn_read_count.load(), torn_rate);
        
        if (torn_rate > 1.0)
        {
            RCLCPP_WARN(get_logger(), "  ⚠️ HIGH TORN READ RATE - Consider adding mutex");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "  ✓ Race condition test PASSED");
        }
    }
    
    // ========== TEST 2: Timing Jitter ==========
    void test_timing_jitter()
    {
        RCLCPP_INFO(get_logger(), "\n--- TEST 2: Timing Jitter ---");
        
        struct TimingStats
        {
            std::string name;
            std::chrono::microseconds target_period;
            std::vector<int64_t> actual_periods_us;
            std::atomic<bool> *running;
        };
        
        std::atomic<bool> running{true};
        std::vector<TimingStats> stats = {
            {"CanReceive (100Hz)", 10000us, {}, &running},
            {"WatchDog (5Hz)", 200000us, {}, &running},
            {"CanInterface (50Hz)", 20000us, {}, &running},
        };
        
        std::vector<std::thread> threads;
        
        for (auto &s : stats)
        {
            threads.emplace_back([&s]() {
                using clock = std::chrono::steady_clock;
                auto last = clock::now();
                auto next = last + s.target_period;
                
                while (*s.running)
                {
                    std::this_thread::sleep_until(next);
                    auto now = clock::now();
                    
                    auto actual = std::chrono::duration_cast<std::chrono::microseconds>(now - last);
                    s.actual_periods_us.push_back(actual.count());
                    
                    last = now;
                    next += s.target_period;
                }
            });
        }
        
        std::this_thread::sleep_for(std::chrono::duration<double>(test_duration_s_));
        running = false;
        
        for (auto &t : threads) t.join();
        
        // Analyze results
        for (const auto &s : stats)
        {
            if (s.actual_periods_us.empty()) continue;
            
            int64_t sum = 0, min_val = INT64_MAX, max_val = 0;
            for (auto p : s.actual_periods_us)
            {
                sum += p;
                min_val = std::min(min_val, p);
                max_val = std::max(max_val, p);
            }
            double avg = static_cast<double>(sum) / s.actual_periods_us.size();
            double target = static_cast<double>(s.target_period.count());
            double jitter_pct = 100.0 * (max_val - min_val) / target;
            
            RCLCPP_INFO(get_logger(), "  %s: avg=%.0fus, min=%ldus, max=%ldus, jitter=%.1f%%",
                        s.name.c_str(), avg, min_val, max_val, jitter_pct);
            
            if (jitter_pct > 50.0)
            {
                RCLCPP_WARN(get_logger(), "    ⚠️ HIGH JITTER - May cause timing issues");
            }
        }
    }
    
    // ========== TEST 3: Sequence Continuity ==========
    void test_sequence_continuity()
    {
        RCLCPP_INFO(get_logger(), "\n--- TEST 3: Sequence Continuity ---");
        
        std::atomic<bool> running{true};
        std::atomic<uint64_t> gaps{0};
        std::atomic<uint64_t> samples{0};
        
        std::thread monitor([&]() {
            uint32_t last_seq = 0;
            bool first = true;
            
            while (running)
            {
                if (auto *state = shmitf_.state())
                {
                    uint32_t seq = state->sequence_id;
                    if (!first && seq != last_seq + 1 && seq != last_seq)
                    {
                        gaps++;
                    }
                    last_seq = seq;
                    first = false;
                    samples++;
                }
                std::this_thread::sleep_for(5ms);
            }
        });
        
        std::this_thread::sleep_for(std::chrono::duration<double>(test_duration_s_));
        running = false;
        monitor.join();
        
        double gap_rate = 100.0 * gaps / std::max(1UL, samples.load());
        RCLCPP_INFO(get_logger(), "  Samples: %lu, Sequence gaps: %lu (%.2f%%)",
                    samples.load(), gaps.load(), gap_rate);
        
        if (gap_rate > 5.0)
        {
            RCLCPP_WARN(get_logger(), "  ⚠️ HIGH GAP RATE - HWI may be dropping updates");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "  ✓ Sequence continuity test PASSED");
        }
    }
    
    // ========== TEST 4: Data Consistency ==========
    void test_data_consistency()
    {
        RCLCPP_INFO(get_logger(), "\n--- TEST 4: Data Consistency (Control -> State) ---");
        
        std::atomic<bool> running{true};
        std::atomic<uint64_t> mismatches{0};
        std::atomic<uint64_t> checks{0};
        
        // Writer: set targets
        std::thread writer([&]() {
            float target = 0.0f;
            while (running)
            {
                if (auto *ctrl = shmitf_.control())
                {
                    ctrl->motion_enable = true;
                    ctrl->axis_count = 1;
                    ctrl->axes[0].action = ControlAction::ClosedLoop;
                    ctrl->axes[0].target = target;
                    target += 0.1f;
                    if (target > 10.0f) target = 0.0f;
                }
                std::this_thread::sleep_for(10ms);
            }
        });
        
        // Reader: verify control block is readable
        std::thread reader([&]() {
            while (running)
            {
                if (auto *ctrl = shmitf_.control())
                {
                    // Just check data is valid
                    float t = ctrl->axes[0].target;
                    if (t < -100.0f || t > 100.0f || std::isnan(t))
                    {
                        mismatches++;
                    }
                    checks++;
                }
                std::this_thread::sleep_for(20ms);
            }
        });
        
        std::this_thread::sleep_for(std::chrono::duration<double>(test_duration_s_));
        running = false;
        
        writer.join();
        reader.join();
        
        double error_rate = 100.0 * mismatches / std::max(1UL, checks.load());
        RCLCPP_INFO(get_logger(), "  Checks: %lu, Invalid data: %lu (%.2f%%)",
                    checks.load(), mismatches.load(), error_rate);
        
        if (error_rate > 0.1)
        {
            RCLCPP_WARN(get_logger(), "  ⚠️ DATA CORRUPTION DETECTED");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "  ✓ Data consistency test PASSED");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThreadTestNode>();
    rclcpp::shutdown();
    return 0;
}
