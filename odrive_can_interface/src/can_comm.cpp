#include "odrive_can_interface/can_comm.hpp"
#include <cstring>
#include <cerrno>
#include <unordered_map>
#include <fcntl.h>
#include <poll.h>

using namespace std;

CANInterface::CANInterface() : can_socket_(-1) {}

CANInterface::~CANInterface()
{
    stopReceive();
    if (can_socket_ >= 0)
    {
        close(can_socket_);
        can_socket_ = -1;
    }
}

bool CANInterface::openInterface(const string &interface)
{
    stopReceive();
    if (can_socket_ >= 0)
    {
        close(can_socket_);
        can_socket_ = -1;
    }

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

    struct can_frame frame{};
    frame.can_id = frame_id & CAN_SFF_MASK; // 11bit
    frame.can_dlc = data.size();
    memset(frame.data, 0, sizeof(frame.data));
    if (!data.empty())
    {
        std::memcpy(frame.data, data.data(), data.size());
    }

    const int nbytes = write(can_socket_, &frame, sizeof(frame));
    if (nbytes != static_cast<int>(sizeof(frame)))
    {
        cerr << "Error: Failed to send CAN frame." << endl;
        cerr << "  Socket: " << can_socket_ << endl;
        cerr << "  Frame ID: 0x" << hex << frame_id << dec << endl;
        cerr << "  Expected bytes: " << sizeof(frame) << endl;
        cerr << "  Actual bytes written: " << nbytes << endl;
        cerr << "  errno: " << errno << " (" << strerror(errno) << ")" << endl;
        return false;
    }

    cout << "Sent CAN frame: ID=0x" << hex << frame_id << dec
         << ", Data=";
    for (size_t i = 0; i < data.size(); ++i)
    {
        cout << hex << static_cast<int>(data[i]) << " ";
    }
    cout << dec << endl;
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
        struct pollfd pfd;
        pfd.fd = can_socket_;
        pfd.events = POLLIN;
        const int poll_ret = poll(&pfd, 1, 10);
        if (poll_ret < 0)
        {
            if (errno == EINTR)
                continue;
            break;
        }
        if (poll_ret == 0 || !(pfd.revents & POLLIN))
        {
            continue;
        }

        latest_frames.clear();
        while (true)
        {
            int nbytes = read(can_socket_, &frame, sizeof(frame));
            if (nbytes < 0)
            {
                if (errno == EINTR)
                    continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;
                return;
            }
            if (nbytes < static_cast<int>(sizeof(frame)))
            {
                cerr << "Error: Incomplete CAN frame received." << endl;
                continue;
            }
            latest_frames[frame.can_id] = frame;
        }

        if (!callback_)
        {
            continue;
        }

        for (const auto &item : latest_frames)
        {
            const auto &frm = item.second;
            uint8_t data[8];
            memcpy(data, frm.data, frm.can_dlc);
            callback_(frm.can_id, data, frm.can_dlc);
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
