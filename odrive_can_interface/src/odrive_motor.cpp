#include "odrive_can_interface/odrive_motor.hpp"
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

    if (cmd == CMD_GET_ENCODER_ESTIMATES && dlc >= 8)
    {
        float pos = 0, vel = 0;
        std::memcpy(&pos, data + 0, 4);
        std::memcpy(&vel, data + 4, 4);
        std::lock_guard<std::mutex> lk(mtx_);
        position_ = pos;
        velocity_ = vel;
    }
}
