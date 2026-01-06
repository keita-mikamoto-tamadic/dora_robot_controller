#ifndef CAN_IF_HPP
#define CAN_IF_HPP

#include <cstdint>
#include <string>
#include <vector>

/**
 * CAN FD interface class for request/response communication
 */
class CanInterface
{
public:
    struct RxFrame
    {
        int64_t timestamp_ns;
        uint32_t arb_id;
        uint8_t len;
        uint8_t data[64];
    };

    /**
     * Constructor
     * @param interface_name CAN interface name (e.g., "can0")
     * @param rx_timeout_us RX timeout in microseconds (per frame)
     */
    CanInterface(const std::string& interface_name, int rx_timeout_us = 1000);

    /**
     * Destructor - closes socket
     */
    ~CanInterface();

    // Disable copy
    CanInterface(const CanInterface&) = delete;
    CanInterface& operator=(const CanInterface&) = delete;

    /**
     * Initialize the CAN interface
     * @return true on success, false on failure
     */
    bool init();

    /**
     * Send a single CAN FD frame (non-blocking, no wait for response)
     * @param tx_arb_id TX arbitration ID
     * @param data Payload data
     * @param len Length of payload data
     * @return true on success
     */
    bool send(uint32_t tx_arb_id, const uint8_t* data, uint8_t len);

    /**
     * Receive responses for multiple axes
     * Waits for TX loopbacks and RX responses, matches by expected response IDs
     * @param expected_ids Expected response arbitration IDs (device_id << 8)
     * @param rx_frames Output: received frames (indexed same as expected_ids)
     * @param wall_clock_time_us Output: wall clock time from start to end of receive (includes timeout waits)
     * @return Number of successfully received frames
     */
    int receiveMultiple(const std::vector<uint32_t>& expected_ids,
                        std::vector<RxFrame>& rx_frames,
                        int64_t& wall_clock_time_us);

private:
    std::string interface_name_;
    int rx_timeout_us_;
    int sock_;
    bool initialized_;

    // TX timestamps for cycle time calculation
    std::vector<int64_t> tx_timestamps_;
};

#endif // CAN_IF_HPP
