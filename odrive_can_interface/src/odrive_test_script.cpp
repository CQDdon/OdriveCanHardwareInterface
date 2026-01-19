// Sample usage:
/*ros2 run odrive_can_interface odrive_test_script --ros-args \
  -p can_port:=can0 \
  -p device_ids:="[0,1,2,3,4,5,6,7]" \
  -p duration_s:=5.0 \
  -p max_velocity:=50.0 \
  -p max_position:=10.0 \
  -p rate_hz:=100.0*/
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <thread>
#include <chrono>
#include "odrive_can_interface/can_comm.hpp"
#include "odrive_can_interface/odrive_motor.hpp"

class OdriveTestScriptNode : public rclcpp::Node
{
public:
    OdriveTestScriptNode() : Node("odrive_test_script")
    {
        RCLCPP_INFO(this->get_logger(), "=== ODrive Test Script Node ===");

        const std::string can_port = this->declare_parameter<std::string>("can_port", "can0");
        duration_s_ = this->declare_parameter<double>("duration_s", 5.0);
        max_velocity_ = this->declare_parameter<double>("max_velocity", 50.0);
        max_position_ = this->declare_parameter<double>("max_position", 10.0);
        rate_hz_ = this->declare_parameter<double>("rate_hz", 100.0);
        const std::vector<int64_t> ids = this->declare_parameter<std::vector<int64_t>>(
            "device_ids", std::vector<int64_t>{0, 1, 2, 3, 4, 5, 6, 7});

        can_interface_ = std::make_shared<CANInterface>();
        if (!can_interface_->openInterface(can_port))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface: %s", can_port.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface opened: %s", can_port.c_str());

        for (const auto id : ids)
        {
            if (id < 0 || id > 63)
            {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid device id: %ld", id);
                continue;
            }
            const auto mode = ((id % 2) == 0) ? OdriveMotor::VELOCITY : OdriveMotor::POSITION;
            auto motor = std::make_shared<OdriveMotor>(
                static_cast<uint8_t>(id), mode, can_interface_.get());
            motors_.push_back({static_cast<int>(id), mode, motor});
            const char *mode_label = mode == OdriveMotor::VELOCITY ? "velocity" : "position";
            RCLCPP_INFO(this->get_logger(), "Created motor for device ID: %ld (%s)", id, mode_label);
        }

        if (motors_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid device IDs configured");
            return;
        }

        can_interface_->startReceive();

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { this->tick(); });

        start_time_ = clock::now();
        RCLCPP_INFO(this->get_logger(), "Script start: even=velocity, odd=position, duration=%.2fs",
                    duration_s_);
    }

    ~OdriveTestScriptNode()
    {
        if (can_interface_)
        {
            can_interface_->stopReceive();
        }
    }

private:
    struct MotorEntry
    {
        int id;
        OdriveMotor::ControlMode mode;
        std::shared_ptr<OdriveMotor> motor;
    };

    using clock = std::chrono::steady_clock;

    std::shared_ptr<CANInterface> can_interface_;
    std::vector<MotorEntry> motors_;
    rclcpp::TimerBase::SharedPtr timer_;

    double duration_s_{5.0};
    double max_velocity_{50.0};
    double max_position_{10.0};
    double rate_hz_{100.0};
    clock::time_point start_time_{};

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
        return peak - peak * ((t - half) / half);  // +peak -> 0
    }

    void tick()
    {
        const double t = std::chrono::duration<double>(clock::now() - start_time_).count();
        const double v = triangle(t, duration_s_, max_velocity_);
        const double p = triangle(t, duration_s_, max_position_);

        for (const auto &entry : motors_)
        {
            float target = 0.0f;
            if (entry.mode == OdriveMotor::VELOCITY)
                target = static_cast<float>(v);
            else if (entry.mode == OdriveMotor::POSITION)
                target = static_cast<float>(p);
            (void)entry.motor->setTarget(target);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdriveTestScriptNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
