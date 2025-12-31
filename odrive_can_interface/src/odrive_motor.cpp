#include "odrive_can_interface/odrive_motor.hpp"
#include <chrono>
#include <cstring>
#include <iostream>

using namespace std;

OdriveMotor::OdriveMotor(uint8_t device_id, ControlMode mode, CANInterface *can_interface)
    : device_id_(device_id), mode_(mode), can_interface_(can_interface) {}

OdriveMotor::~OdriveMotor() {}

bool OdriveMotor::setControlMode(ControlMode mode)
{
    mode_ = mode;
    return true;
}

vector<uint8_t> OdriveMotor::floatToBytes(float value)
{
    vector<uint8_t> bytes(sizeof(float));
    memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

uint32_t OdriveMotor::computeFrameId(uint8_t cmd_id) const
{
    return (static_cast<uint32_t>(device_id_) << 5) | cmd_id;
}

void OdriveMotor::appendFloat(std::vector<uint8_t> &buf, float v)
{
    const size_t off = buf.size();
    buf.resize(off + sizeof(float));
    memcpy(buf.data() + off, &v, sizeof(float));
}

vector<uint8_t> OdriveMotor::u32ToBytes(uint32_t v)
{
    vector<uint8_t> b(4);
    b[0] = static_cast<uint8_t>(v & 0xFF);
    b[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    b[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    b[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
    return b;
}

bool OdriveMotor::sendCommand(uint8_t cmd_id, const std::vector<uint8_t> &data) const
{
    const uint32_t frame_id = computeFrameId(cmd_id);
    if (!can_interface_->sendFrame(frame_id, data))
    {
        std::cerr << "ODrive[" << int(device_id_) << "] send cmd 0x"
                  << std::hex << int(cmd_id) << std::dec << " failed\n";
        return false;
    }
    return true;
}

bool OdriveMotor::fullCalibration()
{
    return sendCommand(CMD_SET_AXIS_STATE, u32ToBytes(AXIS_STATE_FULL_CAL));
}

bool OdriveMotor::idle()
{
    return sendCommand(CMD_SET_AXIS_STATE, u32ToBytes(AXIS_STATE_IDLE));
}

bool OdriveMotor::closeLoopControl()
{
    return sendCommand(CMD_SET_AXIS_STATE, u32ToBytes(AXIS_STATE_CLOSED_LOOP));
}

bool OdriveMotor::clearErrors()
{
    return sendCommand(CMD_CLEAR_ERRORS, {});
}

bool OdriveMotor::setHoming()
{
    return sendCommand(CMD_SET_AXIS_STATE, u32ToBytes(AXIS_STATE_HOMING));
}

bool OdriveMotor::setTarget(float value)
{
    uint8_t cmd_id = 0;
    switch (mode_)
    {
    case VELOCITY:
        cmd_id = CMD_SET_INPUT_VEL;
        break;
    case POSITION:
        cmd_id = CMD_SET_INPUT_POS;
        break;
    case TORQUE:
        cmd_id = CMD_SET_INPUT_TORQUE;
        break;
    }
    return sendCommand(cmd_id, floatToBytes(value));
}

uint8_t OdriveMotor::getDeviceId() const
{
    return device_id_;
}

bool OdriveMotor::getFeedback(float &pos, float &vel) const
{
    lock_guard<mutex> lk(mtx_);
    pos = position_;
    vel = velocity_;
    return true;
}

bool OdriveMotor::getStatus(uint32_t &axis_err,
                            uint8_t &axis_state,
                            uint8_t &motor_err,
                            uint8_t &encoder_err,
                            uint8_t &controller_err,
                            uint8_t &trajectory_done,
                            uint64_t &last_hb_ts) const
{
    lock_guard<mutex> lk(mtx_);
    axis_err = axis_error_;
    axis_state = static_cast<uint8_t>(axis_state_);
    motor_err = motor_error_flag_;
    encoder_err = encoder_error_flag_;
    controller_err = controller_error_flag_;
    trajectory_done = traj_done_ ? 1 : 0;
    last_hb_ts = last_hb_timestamp_ns_;
    return true;
}

float OdriveMotor::getVelocity() const
{
    lock_guard<mutex> lk(mtx_);
    return velocity_;
}

float OdriveMotor::getPosition() const
{
    lock_guard<mutex> lk(mtx_);
    return position_;
}

void OdriveMotor::onCanFeedback(uint32_t frame_id, const uint8_t *data, uint8_t dlc)
{
    const uint32_t axis = (frame_id >> 5);
    const uint8_t cmd = static_cast<uint8_t>(frame_id & 0x1F);
    if (axis != device_id_)
        return;

    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    switch (cmd)
    {
    case CMD_GET_ENCODER_ESTIMATES:
        if (dlc >= 8)
        {
            float pos = 0, vel = 0;
            std::memcpy(&pos, data + 0, 4);
            std::memcpy(&vel, data + 4, 4);
            std::lock_guard<std::mutex> lk(mtx_);
            position_ = pos;
            velocity_ = vel;
        }
        break;

    case CMD_HEARTBEAT:
        if (dlc >= 8)
        {
            uint32_t axis_error = 0;
            uint8_t axis_state = data[4];
            uint8_t motor_err_flag = data[5];
            uint8_t encoder_err_flag = data[6];
            uint8_t controller_err_flag = data[7] & 0x7F; // lower 7 bits
            bool traj_done = (data[7] & 0x80) != 0;       // bit7

            std::memcpy(&axis_error, data + 0, 4);

            std::lock_guard<std::mutex> lk(mtx_);
            axis_error_ = axis_error;
            axis_state_ = axis_state;
            motor_error_flag_ = motor_err_flag;
            encoder_error_flag_ = encoder_err_flag;
            controller_error_flag_ = controller_err_flag;
            traj_done_ = traj_done;
            last_hb_timestamp_ns_ = static_cast<uint64_t>(now_ns);
        }
        break;

    case CMD_GET_MOTOR_ERROR:
        if (dlc >= 8)
        {
            uint64_t motor_error = 0;
            std::memcpy(&motor_error, data + 0, 8);
            std::lock_guard<std::mutex> lk(mtx_);
            motor_error_ = motor_error;
        }
        break;

    case CMD_GET_ENCODER_ERROR:
        if (dlc >= 4)
        {
            uint32_t encoder_error = 0;
            std::memcpy(&encoder_error, data + 0, 4);
            std::lock_guard<std::mutex> lk(mtx_);
            encoder_error_ = encoder_error;
        }
        break;

    case CMD_GET_CONTROLLER_ERROR:
        if (dlc >= 4)
        {
            uint32_t controller_error = 0;
            std::memcpy(&controller_error, data + 0, 4);
            std::lock_guard<std::mutex> lk(mtx_);
            controller_error_ = controller_error;
        }
        break;

    default:
        break;
    }
}

// Default implementations of virtual error callbacks (can be overridden in derived class)
void OdriveMotor::onHeartbeatError(uint32_t axis_error, uint32_t axis_state, uint32_t controller_flags)
{
    // Default: do nothing. Override this in your scope to handle heartbeat errors
}

void OdriveMotor::onMotorError(uint64_t motor_error)
{
    // Default: do nothing. Override this in your scope to handle motor errors
}

void OdriveMotor::onEncoderError(uint32_t encoder_error)
{
    // Default: do nothing. Override this in your scope to handle encoder errors
}

void OdriveMotor::onControllerError(uint32_t controller_error)
{
    // Default: do nothing. Override this in your scope to handle controller errors
}

// Getter methods for error status
uint32_t OdriveMotor::getAxisError() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return axis_error_;
}

uint32_t OdriveMotor::getAxisState() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return axis_state_;
}

uint64_t OdriveMotor::getMotorError() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return motor_error_;
}

uint32_t OdriveMotor::getEncoderError() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return encoder_error_;
}

uint32_t OdriveMotor::getControllerError() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return controller_error_;
}

bool OdriveMotor::getTrajectoryStatus()
{
    std::lock_guard<std::mutex> lk(mtx_);
    return traj_done_;
}
