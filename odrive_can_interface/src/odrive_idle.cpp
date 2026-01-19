#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include "odrive_can_interface/can_comm.hpp"
#include "odrive_can_interface/odrive_motor.hpp"

class OdriveIdleNode : public rclcpp::Node
{
public:
    OdriveIdleNode() : Node("odrive_idle")
    {
        RCLCPP_INFO(this->get_logger(), "=== ODrive Idle Node ===");

        const std::string can_port = this->declare_parameter<std::string>("can_port", "can0");
        const std::vector<int64_t> ids = this->declare_parameter<std::vector<int64_t>>(
            "device_ids", std::vector<int64_t>{0, 1, 2, 3, 4, 5, 6, 7});
        const int settle_ms = this->declare_parameter<int>("settle_ms", 20);

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
            auto motor = std::make_shared<OdriveMotor>(
                static_cast<uint8_t>(id), OdriveMotor::POSITION, can_interface_.get());
            motors_.push_back(motor);
            RCLCPP_INFO(this->get_logger(), "Created motor for device ID: %ld", id);
        }

        if (motors_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid device IDs configured");
            return;
        }

        can_interface_->startReceive();

        for (auto &motor : motors_)
        {
            if (motor->idle())
            {
                RCLCPP_INFO(this->get_logger(), "  Device %d: Idle sent", motor->getDeviceId());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "  Device %d: Failed to send idle", motor->getDeviceId());
            }
            if (settle_ms > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
            }
        }

        RCLCPP_INFO(this->get_logger(), "Idle done. Press Ctrl+C to exit.");
    }

    ~OdriveIdleNode()
    {
        if (can_interface_)
        {
            can_interface_->stopReceive();
        }
    }

private:
    std::shared_ptr<CANInterface> can_interface_;
    std::vector<std::shared_ptr<OdriveMotor>> motors_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdriveIdleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
