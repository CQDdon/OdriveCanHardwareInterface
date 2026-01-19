#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include "odrive_can_interface/can_comm.hpp"
#include "odrive_can_interface/odrive_motor.hpp"

class OdriveCloseLoopNode : public rclcpp::Node
{
public:
    OdriveCloseLoopNode() : Node("odrive_close_loop")
    {
        RCLCPP_INFO(this->get_logger(), "=== ODrive Close Loop Node ===");

        can_interface_ = std::make_shared<CANInterface>();
        if (!can_interface_->openInterface("can0"))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface opened");

        // Tạo các motor cho 4 bánh (POSITION mode để bật closed loop)
        std::vector<uint8_t> device_ids = {0, 1, 2, 3, 4, 5, 6, 7};
        for (auto id : device_ids)
        {
            auto motor = std::make_shared<OdriveMotor>(id, OdriveMotor::POSITION, can_interface_.get());
            motors_.push_back(motor);
            RCLCPP_INFO(this->get_logger(), "Created motor for device ID: %d", id);
        }

        can_interface_->startReceive();
        enableCloseLoop();

        RCLCPP_INFO(this->get_logger(), "Close loop done. Press Ctrl+C to exit.");
    }

    ~OdriveCloseLoopNode()
    {
        if (can_interface_)
        {
            can_interface_->stopReceive();
        }
    }

private:
    std::shared_ptr<CANInterface> can_interface_;
    std::vector<std::shared_ptr<OdriveMotor>> motors_;

    void enableCloseLoop()
    {
        RCLCPP_INFO(this->get_logger(), "Enabling closed loop control...");
        for (auto &motor : motors_)
        {
            if (motor->closeLoopControl())
            {
                RCLCPP_INFO(this->get_logger(), "  Device %d: Closed loop enabled", motor->getDeviceId());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "  Device %d: Failed to enable closed loop", motor->getDeviceId());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdriveCloseLoopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
