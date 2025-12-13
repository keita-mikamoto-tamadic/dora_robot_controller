#include "can_if.hpp"

#include <iostream>
#include <cstring>
#include <algorithm>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <poll.h>

CanInterface::CanInterface(const std::string& interface_name, int rx_timeout_us)
    : interface_name_(interface_name)
    , rx_timeout_us_(rx_timeout_us)
    , sock_(-1)
    , initialized_(false)
{
}

CanInterface::~CanInterface()
{
    if (sock_ >= 0)
    {
        close(sock_);
        sock_ = -1;
    }
}

bool CanInterface::init()
{
    if (initialized_)
    {
        return true;
    }

    // Open SocketCAN
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
    {
        std::cerr << "[CanInterface] Failed to open CAN socket" << std::endl;
        return false;
    }

    // Enable CAN FD
    int enable_canfd = 1;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        std::cerr << "[CanInterface] Failed to enable CAN FD" << std::endl;
        close(sock_);
        sock_ = -1;
        return false;
    }

    // Enable receiving own messages (for TX timestamp)
    int recv_own_msgs = 1;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0)
    {
        std::cerr << "[CanInterface] Warning: Failed to enable CAN_RAW_RECV_OWN_MSGS" << std::endl;
    }

    // Enable software timestamps
    int timestamp_on = 1;
    if (setsockopt(sock_, SOL_SOCKET, SO_TIMESTAMP, &timestamp_on, sizeof(timestamp_on)) < 0)
    {
        std::cerr << "[CanInterface] Warning: Failed to enable SO_TIMESTAMP" << std::endl;
    }

    // Bind to interface
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
    {
        std::cerr << "[CanInterface] Failed to get interface index for " << interface_name_ << std::endl;
        close(sock_);
        sock_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "[CanInterface] Failed to bind CAN socket" << std::endl;
        close(sock_);
        sock_ = -1;
        return false;
    }

    initialized_ = true;
    return true;
}

bool CanInterface::send(uint32_t tx_arb_id, const uint8_t* data, uint8_t len)
{
    if (!initialized_)
    {
        std::cerr << "[CanInterface] Not initialized" << std::endl;
        return false;
    }

    // Build TX frame
    struct canfd_frame tx_frame;
    std::memset(&tx_frame, 0, sizeof(tx_frame));

    if (tx_arb_id > 0x7FF)
    {
        tx_frame.can_id = tx_arb_id | CAN_EFF_FLAG;
    }
    else
    {
        tx_frame.can_id = tx_arb_id;
    }

    tx_frame.len = (len > 64) ? 64 : len;
    tx_frame.flags = CANFD_BRS;
    std::memcpy(tx_frame.data, data, tx_frame.len);

    // Send CAN frame
    ssize_t nbytes = write(sock_, &tx_frame, sizeof(tx_frame));
    if (nbytes < 0)
    {
        std::cerr << "[CanInterface] Failed to send CAN frame" << std::endl;
        return false;
    }

    return true;
}

int CanInterface::receiveMultiple(const std::vector<uint32_t>& expected_ids,
                                   std::vector<RxFrame>& rx_frames,
                                   int64_t& total_cycle_time_us)
{
    if (!initialized_)
    {
        std::cerr << "[CanInterface] Not initialized" << std::endl;
        return 0;
    }

    size_t num_axes = expected_ids.size();
    rx_frames.resize(num_axes);
    for (auto& f : rx_frames)
    {
        f.timestamp_ns = 0;
        f.arb_id = 0;
        f.len = 0;
    }

    // Track TX loopback timestamps for each axis
    std::vector<int64_t> tx_timestamps(num_axes, 0);
    std::vector<bool> got_tx_loopback(num_axes, false);
    std::vector<bool> got_rx_response(num_axes, false);

    // Calculate TX arb IDs (0x8000 | device_id) from expected response IDs (device_id << 8)
    std::vector<uint32_t> tx_arb_ids(num_axes);
    for (size_t i = 0; i < num_axes; ++i)
    {
        uint32_t device_id = expected_ids[i] >> 8;
        tx_arb_ids[i] = 0x8000 | device_id;
    }

    int received_count = 0;
    total_cycle_time_us = 0;

    // Timeout for all frames (axis count * per-frame timeout)
    int total_timeout_ms = static_cast<int>((rx_timeout_us_ * num_axes * 2 + 999) / 1000);
    if (total_timeout_ms < 1) total_timeout_ms = 1;

    // Prepare for recvmsg with timestamp
    struct canfd_frame recv_frame;
    struct iovec iov;
    iov.iov_base = &recv_frame;
    iov.iov_len = sizeof(recv_frame);

    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval))];
    struct msghdr msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrlmsg;
    msg.msg_controllen = sizeof(ctrlmsg);

    struct pollfd pfd;
    pfd.fd = sock_;
    pfd.events = POLLIN;

    // Keep receiving until we get all responses or timeout
    while (static_cast<size_t>(received_count) < num_axes)
    {
        int ret = poll(&pfd, 1, total_timeout_ms);
        if (ret <= 0)
        {
            // Timeout
            break;
        }

        if (pfd.revents & POLLIN)
        {
            msg.msg_controllen = sizeof(ctrlmsg);
            ssize_t rx_bytes = recvmsg(sock_, &msg, 0);

            if (rx_bytes > 0)
            {
                uint32_t rx_arb_id = recv_frame.can_id & CAN_EFF_MASK;

                // Extract timestamp from control message
                struct timeval tv = {0, 0};
                for (struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
                     cmsg != NULL;
                     cmsg = CMSG_NXTHDR(&msg, cmsg))
                {
                    if (cmsg->cmsg_level == SOL_SOCKET &&
                        cmsg->cmsg_type == SO_TIMESTAMP)
                    {
                        std::memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
                        break;
                    }
                }
                int64_t frame_timestamp_ns = tv.tv_sec * 1000000000LL + tv.tv_usec * 1000LL;

                // Check if this is a TX loopback
                for (size_t i = 0; i < num_axes; ++i)
                {
                    if (rx_arb_id == tx_arb_ids[i] && !got_tx_loopback[i])
                    {
                        tx_timestamps[i] = frame_timestamp_ns;
                        got_tx_loopback[i] = true;
                        break;
                    }
                }

                // Check if this is an RX response
                for (size_t i = 0; i < num_axes; ++i)
                {
                    if (rx_arb_id == expected_ids[i] && !got_rx_response[i])
                    {
                        got_rx_response[i] = true;
                        received_count++;

                        // Fill output frame
                        rx_frames[i].timestamp_ns = frame_timestamp_ns;
                        rx_frames[i].arb_id = rx_arb_id;
                        rx_frames[i].len = recv_frame.len;
                        std::memcpy(rx_frames[i].data, recv_frame.data, recv_frame.len);

                        // Calculate cycle time if we have TX timestamp
                        if (got_tx_loopback[i] && tx_timestamps[i] > 0)
                        {
                            int64_t cycle_us = (frame_timestamp_ns - tx_timestamps[i]) / 1000;
                            total_cycle_time_us += cycle_us;
                        }
                        break;
                    }
                }
            }
        }
    }

    return received_count;
}
