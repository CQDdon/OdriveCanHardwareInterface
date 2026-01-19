#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include "odrive_can_interface/can_comm.hpp"
#include "odrive_can_interface/odrive_motor.hpp"

class OdriveSetTargetNode : public rclcpp::Node
{
public:
    OdriveSetTargetNode() : Node("odrive_set_target")
    {
        RCLCPP_INFO(this->get_logger(), "=== ODrive Set Target Node ===");

        can_port_ = this->declare_parameter<std::string>("can_port", "can0");
        value_ = this->declare_parameter<double>("value", 0.0);
        period_ms_ = this->declare_parameter<int>("period_ms", 0);
        const std::string mode = this->declare_parameter<std::string>("mode", "position");
        const std::vector<int64_t> ids = this->declare_parameter<std::vector<int64_t>>(
            "device_ids", std::vector<int64_t>{0, 2});

        if (!parseMode(mode, control_mode_))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode '%s' (use position|velocity|torque)", mode.c_str());
            return;
        }

        can_interface_ = std::make_shared<CANInterface>();
        if (!can_interface_->openInterface(can_port_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface: %s", can_port_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface opened: %s", can_port_.c_str());

        for (const auto id : ids)
        {
            if (id < 0 || id > 63)
            {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid device id: %ld", id);
                continue;
            }
            auto motor = std::make_shared<OdriveMotor>(
                static_cast<uint8_t>(id), control_mode_, can_interface_.get());
            motors_.push_back(motor);
            RCLCPP_INFO(this->get_logger(), "Created motor for device ID: %ld", id);
        }

        if (motors_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid device IDs configured");
            return;
        }

        can_interface_->startReceive();
        sendTargets();

        if (period_ms_ > 0)
        {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms_),
                [this]() { this->sendTargets(); });
            RCLCPP_INFO(this->get_logger(), "Repeating target every %d ms", period_ms_);
        }

        RCLCPP_INFO(this->get_logger(), "Set target done. Press Ctrl+C to exit.");
    }

    ~OdriveSetTargetNode()
    {
        if (can_interface_)
        {
            can_interface_->stopReceive();
        }
    }

private:
    std::shared_ptr<CANInterface> can_interface_;
    std::vector<std::shared_ptr<OdriveMotor>> motors_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string can_port_;
    double value_{0.0};
    int period_ms_{0};
    OdriveMotor::ControlMode control_mode_{OdriveMotor::POSITION};

    static bool parseMode(const std::string &mode, OdriveMotor::ControlMode &out)
    {
        if (mode == "position")
        {
            out = OdriveMotor::POSITION;
            return true;
        }
        if (mode == "velocity")
        {
            out = OdriveMotor::VELOCITY;
            return true;
        }
        if (mode == "torque")
        {
            out = OdriveMotor::TORQUE;
            return true;
        }
        return false;
    }

    void sendTargets()
    {
        for (auto &motor : motors_)
        {
            (void)motor->setTarget(static_cast<float>(value_));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdriveSetTargetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
