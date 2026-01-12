#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <cstdint>
#include <string>
#include <vector>

/**
 * CAN FD interface class for moteus communication
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
     * @param rx_timeout_us RX timeout in microseconds (per group)
     */
    CanInterface(const std::string& interface_name, int rx_timeout_us = 2000);

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
     * Send a single CAN FD frame (non-blocking)
     * @param arb_id Arbitration ID
     * @param data Payload data
     * @param len Length of payload data
     * @return true on success
     */
    bool send(uint32_t arb_id, const uint8_t* data, uint8_t len);

    /**
     * Receive responses for multiple axes
     * @param expected_ids Expected response arbitration IDs (device_id << 8)
     * @param rx_frames Output: received frames
     * @return Number of successfully received frames
     */
    int receiveMultiple(const std::vector<uint32_t>& expected_ids,
                        std::vector<RxFrame>& rx_frames);

private:
    std::string interface_name_;
    int rx_timeout_us_;
    int sock_;
    bool initialized_;
};

#endif // CAN_INTERFACE_HPP
