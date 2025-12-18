#pragma once

#include <vector>
#include <string>
#include <memory>
#include <cstdint>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"



#include "odrive_can_interface/odrive_motor.hpp"
#include "odrive_can_interface/can_comm.hpp"
#include "odrive_can_interface/shared_memory.hpp"

namespace odrive_can_interface
{

    class OdriveCANSystem : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(OdriveCANSystem);

        // Lifecycle
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        // Interfaces
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // I/O
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        rclcpp::Logger logger_{rclcpp::get_logger("OdriveCANSystem")};

        // HW params (from URDF <hardware><param>)
        std::string can_port_{"can0"};
        int baud_rate_{500000};
        bool use_velocity_command_{true};

        // Transport + driver
        std::shared_ptr<CANInterface> can_;
        std::vector<std::shared_ptr<OdriveMotor>> motors_;

        // Buffers ROS 2 Control
        std::vector<double> position_;
        std::vector<double> velocity_;

        std::vector<OdriveMotor::ControlMode> joint_mode_;
        std::vector<double> command_pos_;
        std::vector<double> command_vel_;

    // Shared memory 
        SharedMemoryInterface shmitf_;
        SharedMemorySegment   shmseg_{SHM_CMD, sizeof(ShmCommand)};
    // Threads
        std::thread hsh_to_hwi_thread_;
        std::thread hwi_to_hsh_thread_;
        std::thread can_interface_thread_;

    // Control flag
        std::atomic<bool> running_{false};

    // Thread functions
        void HSH_HWI();
        void HWI_HSH();
        void CanInterface();

    };  


} // namespace odrive_can_interface


