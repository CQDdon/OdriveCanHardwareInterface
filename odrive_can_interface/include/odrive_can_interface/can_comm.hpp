#ifndef CAN_COMM_HPP_
#define CAN_COMM_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <iostream>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <functional>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <array>
#include <mutex>

class CANInterface
{
public:
    using FeedbackCallback = std::function<void(uint32_t frame_id, const uint8_t *data, uint8_t dlc)>;

    CANInterface();
    ~CANInterface();

    bool openInterface(const std::string &interface);
    bool sendFrame(uint32_t frame_id, const std::vector<uint8_t> &data);

    void registerFeedbackCallback(FeedbackCallback cb);
    
    void startReceive();
    void stopReceive();

private:
    bool openSocket(const std::string &interface);
    bool reopenSocket();
    void receiveLoop();
    void startTransmit();
    void stopTransmit();
    void transmitLoop();
    int socketFdSnapshot();
    bool writeFrame(uint32_t frame_id, const uint8_t *data, uint8_t dlc);
    bool waitWritable(int timeout_ms);

private:
    int can_socket_;
    std::string interface_name_;
    FeedbackCallback callback_{};

    std::atomic<bool> receiving_{false};
    std::thread receive_thread_;

    struct TxFrame
    {
        uint32_t id{0};
        uint8_t dlc{0};
        std::array<uint8_t, 8> data{};
    };

    static constexpr size_t kTxQueueMax = 1024;
    std::atomic<bool> transmitting_{false};
    std::thread transmit_thread_;
    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;
    std::deque<TxFrame> tx_queue_;

    std::mutex socket_mutex_;
};

#endif // CAN_COMM_HPP_
