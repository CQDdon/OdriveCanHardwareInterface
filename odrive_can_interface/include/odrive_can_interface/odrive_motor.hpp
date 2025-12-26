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

    // Error handling callbacks - implement these in your scope
    virtual void onHeartbeatError(uint32_t axis_error, uint32_t axis_state, uint32_t controller_flags);
    virtual void onMotorError(uint64_t motor_error);
    virtual void onEncoderError(uint32_t encoder_error);
    virtual void onControllerError(uint32_t controller_error);

    // Get latest error status
    uint32_t getAxisError() const;
    uint32_t getAxisState() const;
    uint64_t getMotorError() const;
    uint32_t getEncoderError() const;
    uint32_t getControllerError() const;
    bool getTrajectoryStatus();


private:
    // CAN Command IDs
    static constexpr uint8_t CMD_SET_AXIS_STATE         = 0x07;
    static constexpr uint8_t CMD_GET_ENCODER_ESTIMATES  = 0x09;
    static constexpr uint8_t CMD_SET_INPUT_POS          = 0x0C;
    static constexpr uint8_t CMD_SET_INPUT_VEL          = 0x0D;
    static constexpr uint8_t CMD_SET_INPUT_TORQUE       = 0x0E;
    static constexpr uint8_t CMD_CLEAR_ERRORS           = 0x18;
    static constexpr uint8_t CMD_HEARTBEAT              = 0x01;
    static constexpr uint8_t CMD_GET_MOTOR_ERROR        = 0x03;
    static constexpr uint8_t CMD_GET_ENCODER_ERROR      = 0x04;
    static constexpr uint8_t CMD_GET_CONTROLLER_ERROR   = 0x1D;
    // Axis state (ODrive)
    static constexpr uint32_t AXIS_STATE_IDLE        = 0x01;
    static constexpr uint32_t AXIS_STATE_CLOSED_LOOP = 0x08;
    static constexpr uint32_t AXIS_STATE_FULL_CAL    = 0x03; 
    static constexpr uint32_t AXIS_STATE_HOMING      = 0x0B; 
    // Axis error
    static constexpr uint32_t INVALID_STATE                = 0x01; 
    static constexpr uint32_t MOTOR_FAILED                 = 0x40; 
    static constexpr uint32_t SENSORLESS_ESTIMATOR_FAILED  = 0x80; 
    static constexpr uint32_t ENCODER_FAILED               = 0x100; 
    static constexpr uint32_t CONTROLLER_FAILED            = 0x200; 
    static constexpr uint32_t WATCHDOG_TIMER_EXPIRED       = 0x800; 
    static constexpr uint32_t MIN_ENDSTOP_PRESSED          = 0x1000; 
    static constexpr uint32_t MAX_ENDSTOP_PRESSED          = 0x2000; 
    static constexpr uint32_t ESTOP_REQUESTED              = 0x4000; 
    static constexpr uint32_t HOMING_WITHOUT_ENDSTOP       = 0x20000; 
    static constexpr uint32_t OVER_TEMP                    = 0x40000; 
    static constexpr uint32_t UNKNOWN_POSITION             = 0x80000; 
    // Motor error
    static constexpr uint64_t PHASE_RESISTANCE_OUT_OF_RANGE       = 0x01;
    static constexpr uint64_t PHASE_INDUCTANCE_OUT_OF_RANGE       = 0x02;
    static constexpr uint64_t DRV_FAULT                           = 0x08;
    static constexpr uint64_t CONTROL_DEADLINE_MISSED             = 0x10;
    static constexpr uint64_t MODULATION_MAGNITUDE                = 0x80;
    static constexpr uint64_t CURRENT_SENSE_SATURATION            = 0x400;
    static constexpr uint64_t CURRENT_LIMIT_VIOLATION             = 0x1000;
    static constexpr uint64_t MODULATION_IS_NAN                   = 0x10000;
    static constexpr uint64_t MOTOR_THERMISTOR_OVER_TEMP          = 0x20000;
    static constexpr uint64_t FET_THERMISTOR_OVER_TEMP            = 0x40000;
    static constexpr uint64_t TIMER_UPDATE_MISSED                 = 0x80000;
    static constexpr uint64_t CURRENT_MEASUREMENT_UNAVAILABLE     = 0x100000;
    // static constexpr uint64_t CONTROLLER_FAILED                   = 0x200000;
    static constexpr uint64_t I_BUS_OUT_OF_RANGE                  = 0x400000;
    static constexpr uint64_t BRAKE_RESISTOR_DISARMED             = 0x800000;
    static constexpr uint64_t SYSTEM_LEVEL                        = 0x1000000;
    static constexpr uint64_t BAD_TIMING                          = 0x2000000;
    static constexpr uint64_t UNKNOWN_PHASE_ESTIMATE              = 0x4000000;
    static constexpr uint64_t UNKNOWN_PHASE_VEL                   = 0x8000000;
    static constexpr uint64_t UNKNOWN_TORQUE                      = 0x10000000;
    static constexpr uint64_t UNKNOWN_CURRENT_COMMAND             = 0x20000000;
    static constexpr uint64_t UNKNOWN_CURRENT_MEASUREMENT         = 0x40000000;
    static constexpr uint64_t UNKNOWN_VBUS_VOLTAGE                = 0x80000000;
    static constexpr uint64_t UNKNOWN_VOLTAGE_COMMAND             = 0x100000000;
    static constexpr uint64_t UNKNOWN_GAINS                       = 0x200000000;
    static constexpr uint64_t CONTROLLER_INITIALIZING             = 0x400000000;
    static constexpr uint64_t UNBALANCED_PHASES                   = 0x800000000;
    // Encoder error
    // static constexpr uint32_t UNSTABLE_GAIN                       = 0x01; 
    static constexpr uint32_t CPR_POLEPAIRS_MISMATCH              = 0x02; 
    static constexpr uint32_t NO_RESPONSE                         = 0x04; 
    static constexpr uint32_t UNSUPPORTED_ENCODER_MODE            = 0x08; 
    static constexpr uint32_t ILLEGAL_HALL_STATE                  = 0x10; 
    static constexpr uint32_t INDEX_NOT_FOUND_YET                 = 0x20; 
    static constexpr uint32_t HALL_NOT_CALIBRATED_YET             = 0x200; 
    // Controller error
    static constexpr uint32_t OVERSPEED                           = 0x01; 
    static constexpr uint32_t INVALID_INPUT_MODE                  = 0x02; 
    static constexpr uint32_t UNSTABLE_GAIN                       = 0x04; 
    static constexpr uint32_t INVALID_MIRROR_AXIS                 = 0x08; 
    static constexpr uint32_t INVALID_LOAD_ENCODER                = 0x10; 
    static constexpr uint32_t INVALID_ESTIMATE                    = 0x20; 
    static constexpr uint32_t INVALID_CIRCULAR_RANGE              = 0x40; 
    static constexpr uint32_t SPINOUT_DETECTED                    = 0x80; 







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
    
    // Error status storage
    uint32_t axis_error_{0};
    uint32_t axis_state_{0};
    uint64_t motor_error_{0};
    uint32_t encoder_error_{0};
    uint32_t controller_error_{0};
    bool traj_done_{false};

};

#endif // ODRIVE_MOTOR_HPP_
