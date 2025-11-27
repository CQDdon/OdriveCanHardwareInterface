#include "odrive_can_interface/odrive_can_system.hpp"

namespace odrive_can_interface
{

  // ========== INIT ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "Parent on_init failed");
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "on_init(): parsing hardware parameters...");

    try
    {
      if (info.hardware_parameters.count("can_port"))
        can_port_ = info.hardware_parameters.at("can_port");
      else
        can_port_ = "can0";

      if (info.hardware_parameters.count("baud_rate"))
        baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
      else
        baud_rate_ = 500000;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(logger_, "Invalid hardware parameter: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const size_t n = info.joints.size();
    if (n == 0)
    {
      RCLCPP_ERROR(logger_, "No joints");
      return hardware_interface::CallbackReturn::ERROR;
    }

    position_.assign(n, 0.0);
    velocity_.assign(n, 0.0);
    command_pos_.assign(n, 0.0);
    command_vel_.assign(n, 0.0);
    joint_mode_.resize(n);

    // Suy ra mode từng joint theo command_interfaces trong URDF
    for (size_t i = 0; i < n; ++i)
    {
      const auto &j = info.joints[i];

      bool has_cmd_pos = false, has_cmd_vel = false;
      for (const auto &ci : j.command_interfaces)
      {
        if (ci.name == hardware_interface::HW_IF_POSITION)
          has_cmd_pos = true;
        if (ci.name == hardware_interface::HW_IF_VELOCITY)
          has_cmd_vel = true;
      }

      if (has_cmd_vel && !has_cmd_pos)
        joint_mode_[i] = OdriveMotor::VELOCITY;
      else if (has_cmd_pos && !has_cmd_vel)
        joint_mode_[i] = OdriveMotor::POSITION;
      else
      {
        RCLCPP_ERROR(logger_, "Joint '%s' must have exactly one command interface (position OR velocity).", j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Kiểm tra state interfaces bắt buộc
      bool has_state_pos = false, has_state_vel = false;
      for (const auto &si : j.state_interfaces)
      {
        if (si.name == hardware_interface::HW_IF_POSITION)
          has_state_pos = true;
        if (si.name == hardware_interface::HW_IF_VELOCITY)
          has_state_vel = true;
      }
      if (!has_state_pos || !has_state_vel)
      {
        RCLCPP_ERROR(logger_, "Joint '%s' must export state interfaces: position + velocity.", j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    motors_.clear();
    motors_.reserve(n);

    RCLCPP_INFO(logger_, "on_init(): joints=%zu (mode per-joint from URDF), can_port=%s baud=%d",
                n, can_port_.c_str(), baud_rate_);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== EXPORT STATE IF ==========
  std::vector<hardware_interface::StateInterface>
  OdriveCANSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> out;
    out.reserve(info_.joints.size() * 2);

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      const auto &jn = info_.joints[i].name;
      out.emplace_back(jn, hardware_interface::HW_IF_POSITION, &position_[i]);
      out.emplace_back(jn, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    }
    return out;
  }

  // ========== EXPORT CMD IF ==========
  std::vector<hardware_interface::CommandInterface>
  OdriveCANSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> out;
    out.reserve(info_.joints.size());

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      const auto &jn = info_.joints[i].name;
      if (joint_mode_[i] == OdriveMotor::VELOCITY)
        out.emplace_back(jn, hardware_interface::HW_IF_VELOCITY, &command_vel_[i]);
      else
        out.emplace_back(jn, hardware_interface::HW_IF_POSITION, &command_pos_[i]);
    }
    return out;
  }

  // ========== CONFIGURE ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_configure(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "on_configure(): opening CAN and instantiating motors...");

    can_ = std::make_shared<CANInterface>();
    if (!can_->openInterface(can_port_))
    {
      RCLCPP_ERROR(logger_, "Failed to open CAN interface: %s", can_port_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_,
                "CAN %s opened (requested baud %d). "
                "NOTE: set bitrate via: sudo ip link set %s type can bitrate %d; sudo ip link set %s up",
                can_port_.c_str(), baud_rate_, can_port_.c_str(), baud_rate_, can_port_.c_str());

    motors_.clear();
    motors_.reserve(info_.joints.size());
    try
    {
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        const auto &joint = info_.joints[i];
        const uint8_t id = static_cast<uint8_t>(std::stoi(joint.parameters.at("id")));
        motors_.emplace_back(std::make_shared<OdriveMotor>(id, joint_mode_[i], can_.get()));
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(logger_, "Invalid joint parameter (id): %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    can_->registerFeedbackCallback([this](uint32_t can_id, const uint8_t *data, uint8_t dlc)
                                   {
    const uint32_t axis = (can_id >> 5);
    for (auto &m : motors_) {
      if (m->getDeviceId() == axis) { m->onCanFeedback(can_id, data, dlc); break; }
    } });

    can_->startReceive();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== ACTIVATE ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_activate(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "on_activate(): enabling drives...");
    for (auto &m : motors_)
      m->clearErrors();
    for (auto &m : motors_)
      m->setTarget(0.0f);

    // Start homing sequence
    // RCLCPP_INFO(logger_, "Starting homing sequence...");
    // for (auto &m : motors_)
    //   if (m->getDeviceId() % 2 != 0)
    //     m->setHoming();

    // Wait for homing to complete (typical homing takes 2-5 seconds)
    // RCLCPP_INFO(logger_, "Waiting for homing to complete...");
    // std::this_thread::sleep_for(std::chrono::seconds(5));

    // Now enable closed loop control
    RCLCPP_INFO(logger_, "Enabling closed loop control...");
    for (auto &m : motors_)
      m->closeLoopControl();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== DEACTIVATE ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "on_deactivate(): disabling drives...");
    for (auto &m : motors_)
    {
      m->setTarget(0.0f);
      m->idle();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== CLEANUP ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "on_cleanup(): releasing resources...");
    for (auto &m : motors_)
    {
      m->clearErrors();
    }
    if (can_)
      can_->stopReceive();
    motors_.clear();
    // can_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== SHUTDOWN ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_shutdown(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "on_shutdown(): powering down hardware...");
    // if (can_)
    //   can_->stopReceive();
    // can_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== READ LOOP ==========
  hardware_interface::return_type OdriveCANSystem::read(
      const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < motors_.size(); ++i)
    {
      position_[i] = /*static_cast<double>(2*3.141592)**/ static_cast<double>(motors_[i]->getPosition());
      velocity_[i] = /*static_cast<double>(2*3.141592)**/ static_cast<double>(motors_[i]->getVelocity());
    }
    return hardware_interface::return_type::OK;
  }

  // ========== WRITE LOOP ==========
  hardware_interface::return_type
  OdriveCANSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < motors_.size(); ++i)
    {
      float val = 0.0f;
      if (joint_mode_[i] == OdriveMotor::VELOCITY)
        val = static_cast<float>(command_vel_[i]);
      else
        val = static_cast<float>(command_pos_[i]);

      val = val / (2 * 3.141592);
      (void)motors_[i]->setTarget(val);
    }
    return hardware_interface::return_type::OK;
  }

} // namespace odrive_can_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(odrive_can_interface::OdriveCANSystem,
                       hardware_interface::SystemInterface)
