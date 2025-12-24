#ifndef SHM_HPP_
#define SHM_HPP_

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// Shared memory names.
constexpr const char *SHM_HWI_CMD_IF = "/hwi_to_hsh_cmd_if";
constexpr const char *SHM_HWI_STATE = "/hwi_to_hsh_state";
constexpr const char *SHM_HSH_CTRL = "/hsh_to_hwi_ctrl";

constexpr size_t SHM_MAX_AXES = 8;

// HWI system state used by HSH/FSM.
enum class SystemState : uint8_t
{
    Error = 0,
    Idle = 1,
    ClosedLoop = 2,
    Calib = 3
};

// Controller command interface type.
enum class CommandInterface : uint8_t
{
    None = 0,
    Position = 1,
    Velocity = 2,
    Effort = 3
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

struct AxisFeedback
{
    uint32_t can_id;
    uint8_t online;
    uint8_t reserved_u8_0;
    uint16_t reserved_u16_0;
    uint32_t error;       // ODrive error (bitmask)
    uint32_t odrive_state; // ODrive state (IDLE, CLOSED_LOOP, ...)

    float position;       // rad or deg (decide one)
    float velocity;       // rad/s or m/s
    uint64_t last_hb_timestamp_ns;

    AxisFeedback()
        : can_id(0), online(0), reserved_u8_0(0), reserved_u16_0(0),
          error(0), odrive_state(0), position(0.0f), velocity(0.0f),
          last_hb_timestamp_ns(0)
    {}
};

struct HwiStateBlock
{
    SystemState system_state;
    uint32_t sequence_id;
    uint64_t timestamp_ns;
    uint8_t axis_count;
    uint8_t reserved_u8_1;
    uint16_t reserved_u16_1;
    AxisFeedback axes[SHM_MAX_AXES];

    HwiStateBlock()
        : system_state(SystemState::Idle),
          sequence_id(0), timestamp_ns(0), axis_count(0),
          reserved_u8_1(0), reserved_u16_1(0)
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
    {}
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
    {}
};

struct HshControlBlock
{
    uint32_t sequence_id;
    uint64_t timestamp_ns;
    uint8_t axis_count;
    uint8_t motion_enable;
    SafetyAction safety_action;
    uint8_t reserved_u8_0;
    AxisControl axes[SHM_MAX_AXES];

    HshControlBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0), motion_enable(0),
          safety_action(SafetyAction::None), reserved_u8_0(0)
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
    template <typename T> T *as() noexcept { return static_cast<T *>(ptr_); }
    template <typename T> const T *as() const noexcept { return static_cast<const T *>(ptr_); }

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
    bool write_control(const HshControlBlock &control);
    void close();
    bool ready() const noexcept;

    HwiCommandIfBlock *cmd_if() noexcept;
    HwiStateBlock *state() noexcept;
    HshControlBlock *control() noexcept;
    const HwiCommandIfBlock *cmd_if() const noexcept;
    const HwiStateBlock *state() const noexcept;
    const HshControlBlock *control() const noexcept;

private:
    SharedMemorySegment cmd_if_segment_{SHM_HWI_CMD_IF, sizeof(HwiCommandIfBlock)};
    SharedMemorySegment state_segment_{SHM_HWI_STATE, sizeof(HwiStateBlock)};
    SharedMemorySegment control_segment_{SHM_HSH_CTRL, sizeof(HshControlBlock)};
};

} // namespace odrive_can_interface
 
#endif // SHM_HPP
