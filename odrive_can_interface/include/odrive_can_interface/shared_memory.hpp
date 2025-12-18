#ifndef SHM_HPP_
#define SHM_HPP_

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <errno.h>

constexpr const char* SHM_CMD = "/hsh_to_hwi_cmd";
constexpr const char* SHM_STATE = "/hwi_to_hsh_state";

constexpr size_t SHM_MAX_AXES = 8 ;

struct AxisRawFeedback {

    uint32_t can_id;
    uint8_t  online;
    uint8_t  reserved_u8_0;
    uint16_t reserved_u16_0;
    uint32_t error; // error code (bitmask từ ODrive)
    uint32_t state; // Odrive state (IDLE, CLOSED_LOOP,...)
    uint32_t controller_status_raw; // optional, tùy bạn map
    
    // Dữ liệu động học đã decode
    float position;            // rad hoặc deg (bạn thống nhất)
    float velocity;            // rad/s hoặc m/s

    // Thời gian
    uint64_t last_hb_timestamp_ns;  // thời điểm nhận heartbeat gần nhất

    AxisRawFeedback()
        : can_id(0), online(0), reserved_u8_0(0), reserved_u16_0(0),
          error(0), state(0), controller_status_raw(0),
          position(0.0f), velocity(0.0f), last_hb_timestamp_ns(0)
    {}
};

struct SharedStateBlock {
    uint32_t sequence_id;
    uint64_t timestamp_ns;
    uint8_t axis_count;
    uint8_t reserved_u8_1;
    uint16_t reserved_u16_1;
    AxisRawFeedback axes[SHM_MAX_AXES];

    SharedStateBlock()
        : sequence_id(0), timestamp_ns(0), axis_count(0), reserved_u8_1(0), reserved_u16_1(0)
    {
        for (auto &axis : axes)
        {
            axis = {};
        }
    }
};

struct ControlCommand { 
    uint32_t sequence_id;       // tăng khi SH gửi lệnh mới 
    uint64_t timestamp_ns; 
    uint8_t axis_count; 

    bool enable;                // motion_allowed 
    uint8_t control_mode;       // IDLE, CLOSED_LOOP, HOMING, ... 

    float wheel_velocity[SHM_MAX_AXES];   // drive setpoint 
    float steer_angle[SHM_MAX_AXES];      // steer setpoint 
 
    uint8_t safety_action;      // 0=normal, 1=soft-stop, 2=emergency 
}; 

// Type aliases for backward compatibility
using ShmCommand = ControlCommand;
using ShmState = SharedStateBlock;

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
    bool write_cmd(const ShmCommand &command);
    bool write_state(const ShmState &state);
    void close();
    bool ready() const noexcept;

    ShmCommand *command() noexcept;
    ShmState *state() noexcept;
    const ShmCommand *command() const noexcept;
    const ShmState *state() const noexcept;

private:
    SharedMemorySegment command_segment_{SHM_CMD, sizeof(ShmCommand)};
    SharedMemorySegment state_segment_{SHM_STATE, sizeof(ShmState)};
};

} // namespace odrive_can_interface
 
#endif // SHM_HPP