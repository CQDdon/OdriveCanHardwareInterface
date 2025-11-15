#ifndef ODRIVE_MOTOR_HPP_
#define ODRIVE_MOTOR_HPP_

#include <cstdint>
#include <vector>
#include <mutex>
#include "odrive_can_interface/can_comm.hpp"

class OdriveMotor
{
public:
    enum ControlMode
    {
        VELOCITY,
        POSITION,
        TORQUE
    };

    OdriveMotor(uint8_t device_id, ControlMode mode, CANInterface *can_interface);
    ~OdriveMotor();

    bool setControlMode(ControlMode mode);

    // Basic commands
    bool fullCalibration();      // cmd 0x07, data=0x03
    bool idle();                 // cmd 0x07, data=0x01
    bool closeLoopControl();     // cmd 0x07, data=0x08
    bool clearErrors();          // cmd 0x18, data=0x00
    bool setHoming();            // cmd 0x07, data=0x0B
    bool setTarget(float value); // cmd based on mode

    // Feedback snapshot (thread-safe)
    bool getFeedback(float &pos, float &vel) const;
    float getVelocity() const;
    float getPosition() const;

    uint8_t getDeviceId() const;

    // Nhận frame từ HWI (demux xong gọi vào đây)
    void onCanFeedback(uint32_t frame_id, const uint8_t *data, uint8_t dlc);

private:
    static constexpr uint8_t CMD_SET_AXIS_STATE         = 0x07;
    static constexpr uint8_t CMD_GET_ENCODER_ESTIMATES  = 0x09;
    static constexpr uint8_t CMD_SET_INPUT_POS          = 0x0C;
    static constexpr uint8_t CMD_SET_INPUT_VEL          = 0x0D;
    static constexpr uint8_t CMD_SET_INPUT_TORQUE       = 0x0E;
    static constexpr uint8_t CMD_CLEAR_ERRORS           = 0x18;

    // Axis state (ODrive)
    static constexpr uint32_t AXIS_STATE_IDLE        = 0x01;
    static constexpr uint32_t AXIS_STATE_CLOSED_LOOP = 0x08;
    static constexpr uint32_t AXIS_STATE_FULL_CAL    = 0x03; 
    static constexpr uint32_t AXIS_STATE_HOMING      = 0x0B; 

    uint32_t computeFrameId(uint8_t cmd_id) const;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t> &data) const;

    static void appendFloat(std::vector<uint8_t>& buf, float v);
    static std::vector<uint8_t> floatToBytes(float value);
    static std::vector<uint8_t> u32ToBytes(uint32_t v);

private:
    uint8_t device_id_;
    ControlMode mode_;
    CANInterface *can_interface_;

    mutable std::mutex mtx_;
    float velocity_{0.0f};
    float position_{0.0f};
};

#endif // ODRIVE_MOTOR_HPP_
