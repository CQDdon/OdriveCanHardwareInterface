// Test script for OdriveCANSystem hardware interface directly
// Usage:
/*ros2 run odrive_can_interface odrive_hwi_test --ros-args \
  -p can_port:=can0 \
  -p device_ids:="[0,1,2,3,4,5,6,7]" \
  -p duration_s:=5.0 \
  -p max_velocity:=50.0 \
  -p max_position:=10.0 \
  -p rate_hz:=50.0*/

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <thread>
#include <chrono>
#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "odrive_can_interface/odrive_can_system.hpp"
#include "odrive_can_interface/shared_memory.hpp"

class OdriveHwiTestNode : public rclcpp::Node
{
public:
    OdriveHwiTestNode() : Node("odrive_hwi_test")
    {
        RCLCPP_INFO(this->get_logger(), "=== ODrive HWI Test Script ===");

        // Declare parameters
        const std::string can_port = this->declare_parameter<std::string>("can_port", "can0");
        duration_s_ = this->declare_parameter<double>("duration_s", 5.0);
        max_velocity_ = this->declare_parameter<double>("max_velocity", 50.0);
        max_position_ = this->declare_parameter<double>("max_position", 10.0);
        rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);
        const std::vector<int64_t> ids = this->declare_parameter<std::vector<int64_t>>(
            "device_ids", std::vector<int64_t>{0, 1, 2, 3, 4, 5, 6, 7});

        // Build HardwareInfo from parameters
        hardware_interface::HardwareInfo hw_info;
        hw_info.name = "odrive_test_system";
        hw_info.type = "system";
        hw_info.hardware_class_type = "odrive_can_interface::OdriveCANSystem";
        hw_info.hardware_parameters["can_port"] = can_port;
        hw_info.hardware_parameters["baud_rate"] = "250000";

        // Create joints
        for (size_t i = 0; i < ids.size(); ++i)
        {
            const int64_t id = ids[i];
            if (id < 0 || id > 63)
            {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid device id: %ld", id);
                continue;
            }

            hardware_interface::ComponentInfo joint;
            joint.name = "joint_" + std::to_string(id);
            joint.parameters["id"] = std::to_string(id);

            // Even = velocity mode, Odd = position mode
            const bool is_velocity = ((id % 2) == 0);
            joint_is_velocity_.push_back(is_velocity);

            // State interfaces (both position and velocity)
            hardware_interface::InterfaceInfo pos_state;
            pos_state.name = hardware_interface::HW_IF_POSITION;
            joint.state_interfaces.push_back(pos_state);

            hardware_interface::InterfaceInfo vel_state;
            vel_state.name = hardware_interface::HW_IF_VELOCITY;
            joint.state_interfaces.push_back(vel_state);

            // Command interface (one only)
            hardware_interface::InterfaceInfo cmd_if;
            if (is_velocity)
            {
                cmd_if.name = hardware_interface::HW_IF_VELOCITY;
            }
            else
            {
                cmd_if.name = hardware_interface::HW_IF_POSITION;
            }
            joint.command_interfaces.push_back(cmd_if);

            hw_info.joints.push_back(joint);
            device_ids_.push_back(static_cast<int>(id));

            const char *mode_label = is_velocity ? "velocity" : "position";
            RCLCPP_INFO(this->get_logger(), "Joint '%s' (id=%ld, mode=%s)",
                        joint.name.c_str(), id, mode_label);
        }

        if (hw_info.joints.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid joints configured");
            return;
        }

        // Create and initialize hardware interface
        hwi_ = std::make_shared<odrive_can_interface::OdriveCANSystem>();

        RCLCPP_INFO(this->get_logger(), "Calling on_init()...");
        auto ret = hwi_->on_init(hw_info);
        if (ret != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "on_init() failed");
            return;
        }

        // Export interfaces
        state_interfaces_ = hwi_->export_state_interfaces();
        command_interfaces_ = hwi_->export_command_interfaces();

        RCLCPP_INFO(this->get_logger(), "Exported %zu state interfaces, %zu command interfaces",
                    state_interfaces_.size(), command_interfaces_.size());

        // Configure
        RCLCPP_INFO(this->get_logger(), "Calling on_configure()...");
        rclcpp_lifecycle::State unconfigured(
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
        ret = hwi_->on_configure(unconfigured);
        if (ret != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "on_configure() failed");
            return;
        }

        // Activate
        RCLCPP_INFO(this->get_logger(), "Calling on_activate()...");
        rclcpp_lifecycle::State inactive(
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
        ret = hwi_->on_activate(inactive);
        if (ret != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "on_activate() failed");
            return;
        }

        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Hardware interface activated successfully!");

        // Open shared memory to write HshControlBlock directly
        if (!shmitf_.open(false))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open shared memory for control");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Shared memory opened for HshControlBlock access");

        // Start timer
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { this->tick(); });

        start_time_ = clock::now();
        RCLCPP_INFO(this->get_logger(), "Test started: duration=%.2fs, rate=%.1fHz",
                    duration_s_, rate_hz_);
    }

    ~OdriveHwiTestNode()
    {
        if (hwi_ && initialized_)
        {
            // Set all axes to idle before deactivating
            if (auto *ctrl = shmitf_.control())
            {
                ctrl->motion_enable = false;
                for (size_t i = 0; i < SHM_MAX_AXES; ++i)
                {
                    ctrl->axes[i].action = ControlAction::Idle;
                    ctrl->axes[i].target = 0.0f;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            RCLCPP_INFO(this->get_logger(), "Deactivating hardware interface...");
            rclcpp_lifecycle::State active(
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
            hwi_->on_deactivate(active);

            RCLCPP_INFO(this->get_logger(), "Cleaning up hardware interface...");
            rclcpp_lifecycle::State inactive(
                lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
            hwi_->on_cleanup(inactive);

            shmitf_.close();
        }
    }

private:
    using clock = std::chrono::steady_clock;

    std::shared_ptr<odrive_can_interface::OdriveCANSystem> hwi_;
    odrive_can_interface::SharedMemoryInterface shmitf_;
    std::vector<hardware_interface::StateInterface> state_interfaces_;
    std::vector<hardware_interface::CommandInterface> command_interfaces_;
    std::vector<int> device_ids_;
    std::vector<bool> joint_is_velocity_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool initialized_{false};
    double duration_s_{5.0};
    double max_velocity_{50.0};
    double max_position_{10.0};
    double rate_hz_{100.0};
    clock::time_point start_time_{};
    uint64_t tick_count_{0};

    static double triangle(double t, double duration, double peak)
    {
        if (duration <= 0.0)
            return -peak;
        const double half = duration / 2.0;
        if (t <= 0.0)
            return -peak;
        if (t >= duration)
            return 0.0;
        if (t <= half)
            return -peak + 2.0 * peak * (t / half);  // -peak -> +peak
        return peak - peak * ((t - half) / half);    // +peak -> 0
    }

    void tick()
    {
        if (!initialized_)
            return;

        const double t = std::chrono::duration<double>(clock::now() - start_time_).count();

        // Read from hardware
        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Duration period(std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / rate_hz_)));
        hwi_->read(now, period);

        // Calculate targets
        const double v = triangle(t, duration_s_, max_velocity_);
        const double p = triangle(t, duration_s_, max_position_);

        // Write directly to HshControlBlock (this is what CanInterface thread reads)
        auto *ctrl = shmitf_.control();
        if (ctrl)
        {
            ctrl->motion_enable = true;
            ctrl->axis_count = static_cast<uint8_t>(device_ids_.size());
            ctrl->timestamp_ns = static_cast<uint64_t>(now.nanoseconds());
            ctrl->sequence_id += 1;

            for (size_t i = 0; i < device_ids_.size() && i < SHM_MAX_AXES; ++i)
            {
                double target = joint_is_velocity_[i] ? v : p;
                // Convert from rad or rad/s to turns or turns/s
                float target_turns = static_cast<float>(target / (2.0 * 3.141592));
                
                ctrl->axes[i].action = ControlAction::ClosedLoop;
                ctrl->axes[i].target = target_turns;
            }
        }

        // Also set command interfaces (for consistency with HWI write path)
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            double target = joint_is_velocity_[i] ? v : p;
            command_interfaces_[i].set_value(target);
        }

        // Write to hardware (this writes to HwiCommandIfBlock, but CanInterface reads HshControlBlock)
        hwi_->write(now, period);

        tick_count_++;

        // Log every 10 ticks (100ms @ 100Hz) to show target changing
        if (tick_count_ % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "[tick %lu] t=%.3fs, vel=%.2f (%.2f turns/s), pos=%.2f (%.2f turns)",
                        tick_count_, t, v, v / (2.0 * 3.141592), p, p / (2.0 * 3.141592));
        }

        // Log feedback periodically (every 1 second)
        static int log_counter = 0;
        if (++log_counter >= static_cast<int>(rate_hz_))
        {
            log_counter = 0;
            RCLCPP_INFO(this->get_logger(), "--- Summary t=%.2fs, vel_cmd=%.2f, pos_cmd=%.2f ---",
                        t, v, p);

            // Print state feedback
            for (size_t i = 0; i < state_interfaces_.size(); i += 2)
            {
                if (i + 1 < state_interfaces_.size())
                {
                    RCLCPP_INFO(this->get_logger(), "  [%s] pos=%.3f, vel=%.3f",
                                state_interfaces_[i].get_prefix_name().c_str(),
                                state_interfaces_[i].get_value(),
                                state_interfaces_[i + 1].get_value());
                }
            }
        }

        // Stop after duration
        if (t >= duration_s_)
        {
            RCLCPP_INFO(this->get_logger(), "Test completed!");
            timer_->cancel();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdriveHwiTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
