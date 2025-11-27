#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include "odrive_can_interface/can_comm.hpp"
#include "odrive_can_interface/odrive_motor.hpp"

class OdriveHomingNode : public rclcpp::Node
{
public:
    OdriveHomingNode() : Node("odrive_homing")
    {
        RCLCPP_INFO(this->get_logger(), "=== ODrive Homing Node ===");
        
        // Mở CAN interface
        can_interface_ = std::make_shared<CANInterface>();
        if (!can_interface_->openInterface("can0"))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface opened");

        // Tạo các motor với ID 1, 3, 5 (POSITION mode cho homing)
        std::vector<uint8_t> device_id_homing = {1, 3, 5};
        
        for (auto id : device_id_homing)
        {
            auto motor = std::make_shared<OdriveMotor>(id, OdriveMotor::POSITION, can_interface_.get());
            motors_.push_back(motor);
            RCLCPP_INFO(this->get_logger(), "Created motor for device ID: %d", id);
        }

        // Bắt đầu receive để xem feedback
        // can_interface_->registerFeedbackCallback(
        //     [this](uint32_t can_id, const uint8_t *data, uint8_t dlc)
        //     {
        //         uint32_t device_id = (can_id >> 5);
        //         RCLCPP_INFO(this->get_logger(), "Received from device %d", device_id);
        //     }
        // );
        can_interface_->startReceive();

        // Thực hiện homing sequence
        executeHoming();

        RCLCPP_INFO(this->get_logger(), "Homing complete. Press Ctrl+C to exit.");
    }

    ~OdriveHomingNode()
    {
        if (can_interface_)
        {
            can_interface_->stopReceive();
        }
    }

private:
    std::shared_ptr<CANInterface> can_interface_;
    std::vector<std::shared_ptr<OdriveMotor>> motors_;

    void executeHoming()
    {
        RCLCPP_INFO(this->get_logger(), "Starting homing sequence...");

        // Bước 1: Clear errors
        RCLCPP_INFO(this->get_logger(), "Step 1: Clearing errors...");
        for (auto &motor : motors_)
        {
            motor->clearErrors();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // Bước 2: Set target về 0
        RCLCPP_INFO(this->get_logger(), "Step 2: Setting target to 0...");
        for (auto &motor : motors_)
        {
            motor->setTarget(0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // Bước 3: Gửi lệnh homing
        RCLCPP_INFO(this->get_logger(), "Step 3: Sending homing command...");
        for (auto &motor : motors_)
        {
            if (motor->setHoming())
            {
                RCLCPP_INFO(this->get_logger(), "  Device %d: Homing started", motor->getDeviceId());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "  Device %d: Failed to start homing", motor->getDeviceId());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Bước 4: Chờ homing hoàn thành (5 giây)
        RCLCPP_INFO(this->get_logger(), "Step 4: Waiting for operation to complete (5 seconds)...");
        for (int i = 1; i <= 5; i++)
        {
            RCLCPP_INFO(this->get_logger(), "  %d/5 seconds...", i);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Bước 5: Bật closed loop control
        RCLCPP_INFO(this->get_logger(), "Step 5: Enabling closed loop control...");
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

        RCLCPP_INFO(this->get_logger(), "=== Testing sequence completed ===");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdriveHomingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}