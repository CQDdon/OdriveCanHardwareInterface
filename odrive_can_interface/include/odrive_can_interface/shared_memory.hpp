#ifndef SHM_HPP_
#define SHM_HPP_

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// Shared memory names.
constexpr const char *SHM_HWI_CMD_IF = "/hwi_to_hsh_cmd_if";
constexpr const char *SHM_HWI_STATE = "/hwi_to_hsh_state"; // lite state
constexpr const char *SHM_HWI_STATE_DEBUG = "/hwi_to_hsh_state_debug";
constexpr const char *SHM_HSH_CTRL = "/hsh_to_hwi_ctrl";

constexpr size_t SHM_MAX_AXES = 8;

// Controller command interface type.
enum class CommandInterface : uint8_t
{
    None = 0,
    Position = 1,
    Velocity = 2,
    Torque = 3
};

// Control action that HSH requests on each axis.
enum class ControlAction : uint8_t
{
    None = 0,
    Idle = 1,
    ClosedLoop = 2,
    FullCalib = 3,
    Homing = 4,
    ClearErrors = 5
};

enum class SafetyAction : uint8_t
{
    None = 0,
    SoftStop = 1,
    Emergency = 2
};

// Compact axis state summary derived from ODrive AxisState.
enum class AxisStateSummary : uint8_t
{
    Unknown = 0,
    Idle = 1,
    Startup = 2,
    Calibrating = 3,
    Homing = 4,
    Running = 5
};

struct AxisFeedbackLite
{
    uint32_t can_id;
    AxisStateSummary odrive_state_summary;
    uint32_t error;
    uint8_t online;
    uint8_t reserved_u8_0;
    uint16_t reserved_u16_0;
    float position; // rad
    float velocity; // rad/s

    AxisFeedbackLite()
        : can_id(0), odrive_state_summary(AxisStateSummary::Unknown), error(0),
          online(0), reserved_u8_0(0), reserved_u16_0(0),
          position(0.0f), velocity(0.0f)
    {
    }
};

struct AxisFeedbackDebug
{
    uint32_t axis_error;
    uint8_t odrive_state;          // Raw AxisState from heartbeat
    uint8_t trajectory_done_flag;  // From heartbeat (bit)
    uint16_t reserved_u16_0;
    uint32_t motor_error;
    uint32_t encoder_error;
    uint32_t controller_error;
    uint64_t last_hb_timestamp_ns;

    AxisFeedbackDebug()
        : axis_error(0), odrive_state(0), trajectory_done_flag(0), reserved_u16_0(0),
          motor_error(0), encoder_error(0), controller_error(0),
          last_hb_timestamp_ns(0)
    {
    }
};

inline bool is_axis_online(const AxisFeedbackLite &ax)
{
    return ax.online != 0;
}

// Map ODrive AxisState to a compact summary used by HSH.
inline AxisStateSummary map_odrive_axis_state_summary(uint8_t axis_state)
{
    switch (axis_state)
    {
    case 1:  // IDLE
        return AxisStateSummary::Idle;
    case 2:  // STARTUP_SEQUENCE
        return AxisStateSummary::Startup;
    case 3:  // FULL_CALIBRATION_SEQUENCE
    case 4:  // MOTOR_CALIBRATION
    case 6:  // ENCODER_INDEX_SEARCH
    case 7:  // ENCODER_OFFSET_CALIBRATION
    case 10: // ENCODER_DIR_FIND
    case 12: // ENCODER_HALL_POLARITY_CALIBRATION
    case 13: // ENCODER_HALL_PHASE_CALIBRATION
        return AxisStateSummary::Calibrating;
    case 11: // HOMING
        return AxisStateSummary::Homing;
    case 8:  // CLOSED_LOOP_CONTROL
    case 9:  // LOCKIN_SPIN
        return AxisStateSummary::Running;
    case 0:  // UNDEFINED
    default:
        return AxisStateSummary::Unknown;
    }
}

struct HwiStateBlock
{
    uint32_t sequence_id;
    uint64_t timestamp_ns;
    uint8_t axis_count;
    uint8_t reserved_u8_1;
    uint16_t reserved_u16_1;
    AxisFeedbackLite axes[SHM_MAX_AXES];

    HwiStateBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0),
          reserved_u8_1(0), reserved_u16_1(0)
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

struct HwiStateDebugBlock
{
    uint32_t sequence_id;
    uint64_t timestamp_ns;
    uint8_t axis_count;
    uint8_t reserved_u8_0;
    uint16_t reserved_u16_0;
    AxisFeedbackDebug axes[SHM_MAX_AXES];

    HwiStateDebugBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0),
          reserved_u8_0(0), reserved_u16_0(0)
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

struct AxisCommandIf
{
    CommandInterface interface;
    uint8_t reserved_u8_0;
    uint16_t reserved_u16_0;
    float value;

    AxisCommandIf()
        : interface(CommandInterface::None),
          reserved_u8_0(0), reserved_u16_0(0), value(0.0f)
    {
    }
};

struct HwiCommandIfBlock
{
    uint32_t sequence_id;
    uint64_t timestamp_ns;
    uint8_t axis_count;
    uint8_t reserved_u8_0;
    uint16_t reserved_u16_0;
    AxisCommandIf axes[SHM_MAX_AXES];

    HwiCommandIfBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0),
          reserved_u8_0(0), reserved_u16_0(0)
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

struct AxisControl
{
    ControlAction action;
    uint8_t reserved_u8_0;
    uint16_t reserved_u16_0;
    float target;

    AxisControl()
        : action(ControlAction::None), reserved_u8_0(0),
          reserved_u16_0(0), target(0.0f)
    {
    }
};

struct HshControlBlock
{
    uint32_t sequence_id;
    uint64_t timestamp_ns;
    uint8_t axis_count;
    uint8_t motion_enable;
    SafetyAction safety_action;
    uint8_t debug_enable;
    AxisControl axes[SHM_MAX_AXES];

    HshControlBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0), motion_enable(0),
          safety_action(SafetyAction::None), debug_enable(0)
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

namespace odrive_can_interface
{

    class SharedMemorySegment
    {
    public:
        SharedMemorySegment() = default;
        SharedMemorySegment(std::string name, std::size_t size);
        ~SharedMemorySegment();

        SharedMemorySegment(const SharedMemorySegment &) = delete;
        SharedMemorySegment &operator=(const SharedMemorySegment &) = delete;
        SharedMemorySegment(SharedMemorySegment &&) noexcept;
        SharedMemorySegment &operator=(SharedMemorySegment &&) noexcept;

        bool open(bool create_if_missing = true);
        void close();
        bool is_open() const noexcept { return ptr_ != nullptr; }
        void *data() noexcept { return ptr_; }
        const void *data() const noexcept { return ptr_; }
        template <typename T>
        T *as() noexcept { return static_cast<T *>(ptr_); }
        template <typename T>
        const T *as() const noexcept { return static_cast<const T *>(ptr_); }

        const std::string &name() const noexcept { return name_; }
        std::size_t size() const noexcept { return size_; }

    private:
        void reset();

        std::string name_;
        std::size_t size_{0};
        int fd_{-1};
        void *ptr_{nullptr};
    };

    class SharedMemoryInterface
    {
    public:
        SharedMemoryInterface() = default;
        ~SharedMemoryInterface();

        SharedMemoryInterface(const SharedMemoryInterface &) = delete;
        SharedMemoryInterface &operator=(const SharedMemoryInterface &) = delete;
        SharedMemoryInterface(SharedMemoryInterface &&) noexcept = default;
        SharedMemoryInterface &operator=(SharedMemoryInterface &&) noexcept = default;

        bool open();
        bool write_cmd_if(const HwiCommandIfBlock &command_if);
        bool write_state(const HwiStateBlock &state);
        bool write_state_debug(const HwiStateDebugBlock &state);
        bool write_control(const HshControlBlock &control);
        void close();
        bool ready() const noexcept;

        HwiCommandIfBlock *cmd_if() noexcept;
        HwiStateBlock *state() noexcept;
        HwiStateDebugBlock *state_debug() noexcept;
        HshControlBlock *control() noexcept;
        const HwiCommandIfBlock *cmd_if() const noexcept;
        const HwiStateBlock *state() const noexcept;
        const HwiStateDebugBlock *state_debug() const noexcept;
        const HshControlBlock *control() const noexcept;

    private:
        SharedMemorySegment cmd_if_segment_{SHM_HWI_CMD_IF, sizeof(HwiCommandIfBlock)};
        SharedMemorySegment state_segment_{SHM_HWI_STATE, sizeof(HwiStateBlock)};
        SharedMemorySegment state_debug_segment_{SHM_HWI_STATE_DEBUG, sizeof(HwiStateDebugBlock)};
        SharedMemorySegment control_segment_{SHM_HSH_CTRL, sizeof(HshControlBlock)};
    };

} // namespace odrive_can_interface

#endif // SHM_HPP
