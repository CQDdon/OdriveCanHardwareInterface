#include "odrive_can_interface/can_comm.hpp"
#include <cstring>
#include <cerrno>
#include <unordered_map>
#include <fcntl.h>
#include <poll.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>

using namespace std;

CANInterface::CANInterface() : can_socket_(-1) {}

CANInterface::~CANInterface()
{
    stopReceive();
    stopTransmit();
    if (can_socket_ >= 0)
    {
        close(can_socket_);
        can_socket_ = -1;
    }
}

bool CANInterface::openInterface(const string &interface)
{
    stopReceive();
    stopTransmit();
    if (can_socket_ >= 0)
    {
        close(can_socket_);
        can_socket_ = -1;
    }

    interface_name_ = interface;
    if (!openSocket(interface_name_))
        return false;
    startTransmit();
    return true;
}

bool CANInterface::openSocket(const string &interface)
{
    std::lock_guard<std::mutex> lk(socket_mutex_);
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
        cerr << "Error: could not create CAN socket." << endl;
        return false;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        cerr << "Error: could not get interface index for " << interface << endl;
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    int recv_own = 0;
    (void)setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));
    int snd_buf = 1 << 20;
    int rcv_buf = 1 << 20;
    (void)setsockopt(can_socket_, SOL_SOCKET, SO_SNDBUF, &snd_buf, sizeof(snd_buf));
    (void)setsockopt(can_socket_, SOL_SOCKET, SO_RCVBUF, &rcv_buf, sizeof(rcv_buf));

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        cerr << "Error: could not bind socket to interface " << interface << endl;
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    const int flags = fcntl(can_socket_, F_GETFL, 0);
    if (flags >= 0)
    {
        if (fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK) < 0)
        {
            cerr << "Warning: failed to set CAN socket non-blocking mode." << endl;
        }
    }
    else
    {
        cerr << "Warning: failed to get CAN socket flags." << endl;
    }

    cout << "Socket successfully bound to " << interface << endl;
    return true;
}

bool CANInterface::sendFrame(uint32_t frame_id, const vector<uint8_t> &data)
{
    if (can_socket_ < 0)
    {
        cerr << "Error: Socket is not open. Call openInterface() first." << endl;
        return false;
    }

    if (data.size() > 8)
    {
        cerr << "Error: CAN frame data length exceeds 8 bytes." << endl;
        return false;
    }

    TxFrame frame{};
    frame.id = frame_id & CAN_SFF_MASK;
    frame.dlc = static_cast<uint8_t>(data.size());
    if (!data.empty())
    {
        std::copy_n(data.begin(), frame.dlc, frame.data.begin());
    }
    if (enqueueFrame(frame))
        tx_cv_.notify_one();
    return true;
}

void CANInterface::registerFeedbackCallback(FeedbackCallback cb)
{
    callback_ = move(cb);
}

void CANInterface::receiveLoop()
{
    if (can_socket_ < 0)
    {
        cerr << "Error: Socket is not open. Call openInterface() first." << endl;
        return;
    }

    struct can_frame frame;
    std::unordered_map<uint32_t, struct can_frame> latest_frames;
    while (receiving_)
    {
        const int fd = socketFdSnapshot();
        if (fd < 0)
        {
            if (!reopenSocket())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            continue;
        }

        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int poll_ret = poll(&pfd, 1, 10);
        if (poll_ret < 0)
        {
            if (errno == EINTR)
                continue;
            cerr << "Error: CAN poll failed (" << errno << ": " << strerror(errno) << "). Reopening..." << endl;
            if (!reopenSocket())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            continue;
        }
        if (poll_ret == 0 || !(pfd.revents & POLLIN))
        {
            continue;
        }

        latest_frames.clear();
        while (true)
        {
            const int nbytes = read(fd, &frame, sizeof(frame));
            if (nbytes < 0)
            {
                if (errno == EINTR)
                    continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;
                cerr << "Error: CAN read failed (" << errno << ": " << strerror(errno) << "). Reopening..." << endl;
                if (!reopenSocket())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                break;
            }
            if (nbytes < static_cast<int>(sizeof(frame)))
            {
                cerr << "Error: Incomplete CAN frame received." << endl;
                continue;
            }
            const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
            latest_frames[can_id] = frame;
        }

        if (!callback_)
        {
            continue;
        }

        for (const auto &item : latest_frames)
        {
            const uint32_t can_id = item.first;
            const auto &frm = item.second;
            uint8_t data[8];
            memcpy(data, frm.data, frm.can_dlc);
            callback_(can_id, data, frm.can_dlc);
        }
    }
}

void CANInterface::startReceive()
{
    if (receiving_.exchange(true))
        return;
    receive_thread_ = thread([this]()
                             { this->receiveLoop(); });
}

void CANInterface::stopReceive()
{
    if (!receiving_.exchange(false))
        return;
    if (can_socket_ >= 0)
        shutdown(can_socket_, SHUT_RD);
    if (receive_thread_.joinable())
        receive_thread_.join();
}

void CANInterface::startTransmit()
{
    if (transmitting_.exchange(true))
        return;
    transmit_thread_ = thread([this]()
                              { this->transmitLoop(); });
}

void CANInterface::stopTransmit()
{
    if (!transmitting_.exchange(false))
        return;
    tx_cv_.notify_all();
    if (transmit_thread_.joinable())
        transmit_thread_.join();
}

void CANInterface::transmitLoop()
{
    while (transmitting_)
    {
        TxFrame frame{};
        {
            std::unique_lock<std::mutex> lk(tx_mutex_);
            tx_cv_.wait(lk, [this]()
                        { return !transmitting_ || !tx_queue_.empty(); });
            if (!transmitting_)
                return;
            frame = tx_queue_.front();
            tx_queue_.pop_front();
        }

        const WriteStatus status = writeFrame(frame.id, frame.data.data(), frame.dlc);
        if (status == WriteStatus::kOk)
            continue;
        if (status == WriteStatus::kRetry)
        {
            if (frame.retry_count < kTxRetryMax)
            {
                ++frame.retry_count;
                const int backoff_ms = std::min<int>(kTxRetryBackoffMaxMs, 1 << std::min<int>(frame.retry_count, 6));
                std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
                if (requeueFrameForRetry(frame))
                    tx_cv_.notify_one();
            }
            else
            {
                cerr << "Warning: dropping CAN frame after retry limit." << endl;
            }
        }
    }
}

bool CANInterface::waitWritable(int timeout_ms)
{
    const int fd = socketFdSnapshot();
    if (fd < 0)
        return false;
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLOUT;
    const int ret = poll(&pfd, 1, timeout_ms);
    return ret > 0 && (pfd.revents & POLLOUT);
}

CANInterface::WriteStatus CANInterface::writeFrame(uint32_t frame_id, const uint8_t *data, uint8_t dlc)
{
    const int fd = socketFdSnapshot();
    if (fd < 0)
        return WriteStatus::kRetry;
    struct can_frame frame{};
    frame.can_id = frame_id & CAN_SFF_MASK;
    frame.can_dlc = dlc;
    memset(frame.data, 0, sizeof(frame.data));
    if (dlc > 0)
    {
        std::memcpy(frame.data, data, dlc);
    }

    for (int attempt = 0; attempt < 5; ++attempt)
    {
        const int nbytes = write(fd, &frame, sizeof(frame));
        if (nbytes == static_cast<int>(sizeof(frame)))
        {
            return WriteStatus::kOk;
        }
        if (nbytes < 0 && (errno == ENOBUFS || errno == EAGAIN))
        {
            (void)waitWritable(10);
            continue;
        }
        if (nbytes < 0 && errno == EINTR)
        {
            continue;
        }
        cerr << "Error: Failed to send CAN frame (non-fatal)." << endl;
        cerr << "  Socket: " << fd << endl;
        cerr << "  Frame ID: 0x" << hex << frame_id << dec << endl;
        cerr << "  errno: " << errno << " (" << strerror(errno) << ")" << endl;
        return WriteStatus::kFatal;
    }
    return WriteStatus::kRetry;
}

int CANInterface::socketFdSnapshot()
{
    std::lock_guard<std::mutex> lk(socket_mutex_);
    return can_socket_;
}

bool CANInterface::reopenSocket()
{
    if (interface_name_.empty())
        return false;

    {
        std::lock_guard<std::mutex> lk(socket_mutex_);
        if (can_socket_ >= 0)
        {
            close(can_socket_);
            can_socket_ = -1;
        }
    }

    while (receiving_)
    {
        if (openSocket(interface_name_))
        {
            cerr << "CAN socket reopened on " << interface_name_ << endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

bool CANInterface::framesEqual(const TxFrame &a, const TxFrame &b)
{
    if (a.id != b.id || a.dlc != b.dlc)
        return false;
    return std::memcmp(a.data.data(), b.data.data(), a.dlc) == 0;
}

bool CANInterface::isSetTargetCommand(uint32_t frame_id)
{
    const uint8_t cmd = static_cast<uint8_t>(frame_id & 0x1F);
    return cmd == 0x0C || cmd == 0x0D || cmd == 0x0E;
}

bool CANInterface::floatDeltaSmall(const TxFrame &a, const TxFrame &b)
{
    if (a.dlc != b.dlc || a.dlc < sizeof(float))
        return false;
    if (a.dlc > sizeof(float))
    {
        const size_t tail = a.dlc - sizeof(float);
        if (std::memcmp(a.data.data() + sizeof(float), b.data.data() + sizeof(float), tail) != 0)
            return false;
    }
    float va = 0.0f;
    float vb = 0.0f;
    std::memcpy(&va, a.data.data(), sizeof(float));
    std::memcpy(&vb, b.data.data(), sizeof(float));
    return std::fabs(va - vb) <= kTargetEpsilon;
}

bool CANInterface::enqueueFrame(const TxFrame &frame)
{
    std::lock_guard<std::mutex> lk(tx_mutex_);
#if 0
    // TEMP: disable duplicate/epsilon filter for testing.
    const auto last_it = last_desired_frames_.find(frame.id);
    if (last_it != last_desired_frames_.end())
    {
        if (framesEqual(last_it->second, frame))
            return false;
        if (isSetTargetCommand(frame.id) && floatDeltaSmall(last_it->second, frame))
            return false;
    }
#endif

    last_desired_frames_[frame.id] = frame;

    for (auto it = tx_queue_.begin(); it != tx_queue_.end(); ++it)
    {
        if (it->id == frame.id)
        {
            tx_queue_.erase(it);
            break;
        }
    }
    if (tx_queue_.size() >= kTxQueueMax)
    {
        tx_queue_.pop_front();
    }
    tx_queue_.push_back(frame);
    return true;
}

bool CANInterface::requeueFrameForRetry(const TxFrame &frame)
{
    std::lock_guard<std::mutex> lk(tx_mutex_);
    const auto last_it = last_desired_frames_.find(frame.id);
    if (last_it == last_desired_frames_.end())
        return false;
    if (!framesEqual(last_it->second, frame))
        return false;

    for (const auto &queued : tx_queue_)
    {
        if (queued.id == frame.id)
            return false;
    }

    if (tx_queue_.size() >= kTxQueueMax)
    {
        tx_queue_.pop_front();
    }
    tx_queue_.push_back(frame);
    return true;
}
