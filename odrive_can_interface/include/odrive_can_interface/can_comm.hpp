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
    void receiveLoop();

private:
    int can_socket_;
    FeedbackCallback callback_{};

    std::atomic<bool> receiving_{false};
    std::thread receive_thread_;
};

#endif // CAN_COMM_HPP_
