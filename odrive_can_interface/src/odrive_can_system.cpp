#include "odrive_can_interface/odrive_can_system.hpp"

namespace odrive_can_interface
{
  constexpr uint64_t kHbOnlineTimeoutNs = 200000000ULL; // 200ms, TODO: make configurable

  // ========== INIT ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    // ------INITIALIZE SHM ------
    RCLCPP_INFO(logger_, "Creating shared memory");
    if (!shmitf_.open(true))
    {
      RCLCPP_ERROR(logger_, "Could not create shm interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "SHM interface created successfully");
    // ------INITIALIZE HWI ------
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
    last_axis_action_.assign(n, ControlAction::None);

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
    HwiStateBlock st{};
    st.axis_count = static_cast<uint8_t>(std::min(info_.joints.size(), static_cast<size_t>(SHM_MAX_AXES)));

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      const auto &joint = info_.joints[i];
      st.axes[i].can_id = static_cast<uint32_t>(std::stoi(joint.parameters.at("id")));
    }

    if (!shmitf_.write_state(st))
    {
      RCLCPP_WARN(logger_, "Failed to write initial state to shared memory");
    }
    else
    {
      RCLCPP_INFO(logger_, "Axis count advertised via SHM: %u", st.axis_count);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== ACTIVATE ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_activate(
      const rclcpp_lifecycle::State &)
  {
    deactivating_ = false;
    running_ = true;

    if (!shmitf_.ready())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto *cmd_if = shmitf_.cmd_if();
    if (!cmd_if)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (auto *state_ = shmitf_.state())
    {
      RCLCPP_INFO(logger_, "Sequence : %u, Axis count : %u",
                  state_->sequence_id,
                  state_->axis_count);

      for (size_t i = 0; i < state_->axis_count; ++i)
      {
        RCLCPP_INFO(logger_, "Axis[%u], state = %u, can_id = %u",
                    static_cast<unsigned>(i),
                    static_cast<uint8_t>(state_->axes[i].odrive_state_summary),
                    state_->axes[i].can_id);
      }
    }

    HwiCommandIfBlock out = *cmd_if;
    out.axis_count = static_cast<uint8_t>(
        std::min(info_.joints.size(), static_cast<size_t>(SHM_MAX_AXES)));
    out.sequence_id += 1;

    for (size_t i = 0; i < out.axis_count; ++i)
    {
      if (joint_mode_[i] == OdriveMotor::VELOCITY)
      {
        out.axes[i].interface = CommandInterface::Velocity;
      }
      else
      {
        out.axes[i].interface = CommandInterface::Position;
      }
      out.axes[i].value = 0.0f;
    }

    for (size_t i = out.axis_count; i < SHM_MAX_AXES; ++i)
    {
      out.axes[i].interface = CommandInterface::None;
      out.axes[i].value = 0.0f;
    }

    (void)shmitf_.write_cmd_if(out);

    if (fatal_error_)
    {
      RCLCPP_ERROR(logger_, "Hardware fatal error: %s",
                   last_error_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Start threads
    can_receive_thread_ = std::thread(&OdriveCANSystem::CanReceive, this);
    can_interface_thread_ = std::thread(&OdriveCANSystem::CanInterface, this);
    watch_dog_thread_ = std::thread(&OdriveCANSystem::WatchDog, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait for all threads to start

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== DEACTIVATE ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface...");
    shmitf_.close();
    deactivating_ = true;
    running_ = false;

    if (can_receive_thread_.joinable())
      can_receive_thread_.join();
    if (can_interface_thread_.joinable())
      can_interface_thread_.join();
    if (watch_dog_thread_.joinable())
      watch_dog_thread_.join();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== CLEANUP ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "on_cleanup(): releasing resources...");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== SHUTDOWN ==========
  hardware_interface::CallbackReturn OdriveCANSystem::on_shutdown(
      const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "on_shutdown(): powering down hardware...");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========== READ LOOP ==========
  hardware_interface::return_type OdriveCANSystem::read(
      const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < motors_.size(); ++i)
    {
      position_[i] = static_cast<double>(2 * 3.141592) *
                     static_cast<double>(motors_[i]->getPosition());
      velocity_[i] = static_cast<double>(2 * 3.141592) *
                     static_cast<double>(motors_[i]->getVelocity());
    }
    return hardware_interface::return_type::OK;
  }

  // ========== WRITE LOOP ==========
  hardware_interface::return_type
  OdriveCANSystem::write(const rclcpp::Time &time, const rclcpp::Duration &)
  {
    auto *cmd_if = shmitf_.cmd_if();
    if (!cmd_if)
    {
      return hardware_interface::return_type::ERROR;
    }

    HwiCommandIfBlock out = *cmd_if;
    out.axis_count = static_cast<uint8_t>(
        std::min(info_.joints.size(), static_cast<size_t>(SHM_MAX_AXES)));
    out.timestamp_ns = static_cast<uint64_t>(time.nanoseconds());
    out.sequence_id += 1;

    for (size_t i = 0; i < out.axis_count; ++i)
    {
      if (joint_mode_[i] == OdriveMotor::VELOCITY)
      {
        out.axes[i].interface = CommandInterface::Velocity;
        out.axes[i].value = static_cast<float>(command_vel_[i]);
      }
      else
      {
        out.axes[i].interface = CommandInterface::Position;
        out.axes[i].value = static_cast<float>(command_pos_[i]);
      }
    }

    (void)shmitf_.write_cmd_if(out);
    return hardware_interface::return_type::OK;
  }

  // ========== CAN RECEIVE THREAD ==========
  void OdriveCANSystem::CanReceive()
  {
    RCLCPP_INFO(can_receive_logger_, "CAN RECEIVE thread started");
    auto next_time = clock::now();
    while (running_)
    {
      next_time += RX_FRE;
      if (auto *state_ = shmitf_.state())
      {
        auto *state_dbg = shmitf_.state_debug();
        const auto *ctrl = shmitf_.control();
        const bool debug_enabled = (ctrl && ctrl->debug_enable != 0);
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                clock::now().time_since_epoch())
                                .count();
        state_->timestamp_ns = static_cast<uint64_t>(now_ns);
        state_->sequence_id += 1;
        state_->axis_count = static_cast<uint8_t>(
            std::min(motors_.size(), static_cast<size_t>(SHM_MAX_AXES)));

        for (size_t i = 0; i < state_->axis_count; ++i)
        {
          state_->axes[i].position = static_cast<float>(2 * 3.141592) *
                                     static_cast<float>(motors_[i]->getPosition());
          state_->axes[i].velocity = static_cast<float>(2 * 3.141592) *
                                     static_cast<float>(motors_[i]->getVelocity());

          uint32_t axis_err = 0;
          uint8_t axis_state = 0, traj_done = 0;
          uint64_t motor_err = 0;
          uint32_t encoder_err = 0, controller_err = 0;
          uint64_t last_hb_ts = 0;
          motors_[i]->getStatus(axis_err, axis_state, motor_err, encoder_err, controller_err, traj_done, last_hb_ts);

          state_->axes[i].error = axis_err;
          state_->axes[i].odrive_state_summary = map_odrive_axis_state_summary(axis_state);
          state_->axes[i].online = (last_hb_ts != 0 &&
                                    (static_cast<uint64_t>(now_ns) - last_hb_ts) <= kHbOnlineTimeoutNs)
                                       ? 1
                                       : 0;

          if (debug_enabled && state_dbg)
          {
            state_dbg->sequence_id = state_->sequence_id;
            state_dbg->timestamp_ns = state_->timestamp_ns;
            state_dbg->axis_count = state_->axis_count;
            state_dbg->axes[i].axis_error = axis_err;
            state_dbg->axes[i].odrive_state = axis_state;
            state_dbg->axes[i].motor_error = static_cast<uint32_t>(motor_err);
            state_dbg->axes[i].encoder_error = encoder_err;
            state_dbg->axes[i].controller_error = controller_err;
            state_dbg->axes[i].trajectory_done_flag = traj_done;
            state_dbg->axes[i].last_hb_timestamp_ns = last_hb_ts;
          }
        }
      }
      else
      {
        RCLCPP_ERROR(can_receive_logger_, "Failed to read state from shared memory");
        fatal_error_ = true;
        running_ = false; // stop all threads
        return;
      }
      std::this_thread::sleep_until(next_time);
    }
    RCLCPP_INFO(can_receive_logger_, "CAN RECEIVE thread stopped");
  }

  // ========== WATCH DOG THREAD==========
  void OdriveCANSystem::WatchDog()
  {
    RCLCPP_INFO(watch_dog_logger_, "Watch Dog thread started");
    auto next_time = clock::now();
    while (running_)
    {
      next_time += WATCH_DOG_FRE;
      auto *state_ = shmitf_.state();
      auto *state_dbg = shmitf_.state_debug();
      if (!state_)
      {
        RCLCPP_ERROR(watch_dog_logger_, "Failed to write state to shared memory");
        fatal_error_ = true;
        running_ = false; // stop all threads
        return;
      }

      for (size_t i = 0; i < state_->axis_count; ++i)
      {
        const uint8_t axis_state = static_cast<uint8_t>(motors_[i]->getAxisState());
        state_->axes[i].odrive_state_summary = map_odrive_axis_state_summary(axis_state);
        state_->axes[i].error = 0;
        state_->axes[i].error |= (motors_[i]->getAxisError() & 0xFF) << 0;
        state_->axes[i].error |= (motors_[i]->getMotorError() & 0xFF) << 8;
        state_->axes[i].error |= (motors_[i]->getEncoderError() & 0xFF) << 16;
        state_->axes[i].error |= (motors_[i]->getControllerError() & 0xFF) << 24;
        if (state_dbg)
        {
          state_dbg->axes[i].axis_error = state_->axes[i].error;
          state_dbg->axes[i].odrive_state = axis_state;
          state_dbg->axes[i].motor_error = static_cast<uint32_t>(motors_[i]->getMotorError());
          state_dbg->axes[i].encoder_error = motors_[i]->getEncoderError();
          state_dbg->axes[i].controller_error = motors_[i]->getControllerError();
        }
      }

      std::this_thread::sleep_until(next_time);
    }
    RCLCPP_INFO(watch_dog_logger_, "Watch Dog thread stopped");
  }
  // ========== CAN TRANSMIT THREAD==========
  void OdriveCANSystem::CanInterface()
  {
    RCLCPP_INFO(can_transmit_logger_, "CAN Transmit thread started");
    RCLCPP_INFO(can_transmit_logger_, "on_configure(): opening CAN and instantiating motors...");

    can_ = std::make_shared<CANInterface>();
    if (!can_->openInterface(can_port_))
    {
      RCLCPP_ERROR(can_transmit_logger_, "Failed to open CAN interface: %s", can_port_.c_str());
      fatal_error_ = true;
      running_ = false; // stop all threads
      auto *state_ = shmitf_.state();
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        state_->axes[i].odrive_state_summary = AxisStateSummary::Unknown;
      }
      return;
    }
    RCLCPP_INFO(can_transmit_logger_,
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
      RCLCPP_ERROR(can_transmit_logger_, "Invalid joint parameter (id): %s", e.what());
      fatal_error_ = true;
      running_ = false; // stop all threads
      return;
    }

    can_->registerFeedbackCallback([this](uint32_t can_id, const uint8_t *data, uint8_t dlc)
                                   {
    const uint32_t axis = (can_id >> 5);
    for (auto &m : motors_) {
      if (m->getDeviceId() == axis) { m->onCanFeedback(can_id, data, dlc); break; }
    } });

    can_->startReceive();

    auto next_time = clock::now();
    while (running_ || deactivating_)
    {
      next_time += TX_FRE;
      if (shmitf_.ready())
      {
        const auto *ctrl_ptr = shmitf_.control();
        if (!ctrl_ptr)
        {
          std::this_thread::sleep_until(next_time);
          continue;
        }

        const HshControlBlock ctrl = *ctrl_ptr;
        const auto *state_ptr = shmitf_.state();
        const size_t axis_count = std::min(
            static_cast<size_t>(ctrl.axis_count), motors_.size());
        const size_t action_count = std::min(axis_count, last_axis_action_.size());

        if (deactivating_)
        {
          for (size_t i = 0; i < axis_count; ++i)
          {
            (void)motors_[i]->idle();
            last_axis_action_[i] = ControlAction::Idle;
          }
          deactivating_ = false;
          running_ = false;
          std::this_thread::sleep_until(next_time);
          continue;
        }

        for (size_t i = 0; i < axis_count; ++i)
        {
          const auto action = ctrl.axes[i].action;
          if (state_ptr && !is_axis_online(state_ptr->axes[i]))
          {
            last_axis_action_[i] = ControlAction::None;
            continue;
          }
          ControlAction effective_action = action;
          if (!ctrl.motion_enable && action == ControlAction::ClosedLoop)
          {
            effective_action = ControlAction::Idle;
          }

          bool send_action = false;
          if (i < action_count)
          {
            if (effective_action == ControlAction::None)
            {
              last_axis_action_[i] = ControlAction::None;
            }
            else if (last_axis_action_[i] != effective_action)
            {
              last_axis_action_[i] = effective_action;
              send_action = true;
            }
          }

          if (send_action)
          {
            switch (effective_action)
            {
            case ControlAction::Idle:
              (void)motors_[i]->idle();
              break;
            case ControlAction::ClosedLoop:
              (void)motors_[i]->closeLoopControl();
              break;
            case ControlAction::FullCalib:
              (void)motors_[i]->fullCalibration();
              break;
            case ControlAction::Homing:
              (void)motors_[i]->setHoming();
              break;
            case ControlAction::ClearErrors:
              (void)motors_[i]->clearErrors();
              break;
            case ControlAction::None:
            default:
              break;
            }
          }

          if (ctrl.motion_enable &&
              (ctrl.axes[i].action == ControlAction::None ||
               ctrl.axes[i].action == ControlAction::ClosedLoop))
          {
            float val = static_cast<float>(ctrl.axes[i].target);
            (void)motors_[i]->setTarget(val);
          }
        }
      }
      std::this_thread::sleep_until(next_time);
    }

    RCLCPP_INFO(can_transmit_logger_, "CAN Transmit thread stopped");
  }

} // namespace odrive_can_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(odrive_can_interface::OdriveCANSystem,
                       hardware_interface::SystemInterface)
