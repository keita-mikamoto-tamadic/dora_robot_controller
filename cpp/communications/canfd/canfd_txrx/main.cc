extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <sched.h>
#include <chrono>

#include "can_if.hpp"

// Static axis configuration (set once on axis_config input)
static std::vector<uint8_t> g_device_ids;
static bool g_config_received = false;

// Group boundary: device_id < GROUP_BOUNDARY goes to group1 (右足), >= GROUP_BOUNDARY goes to group2 (左足)
// mimic_v2.json: 右足 = 50,60,70、左足 = 80,90,100
static constexpr uint8_t GROUP_BOUNDARY = 80;

// Parsed frame structure
struct TxFrame {
    uint32_t arb_id;
    uint8_t len;
    uint8_t data[64];
    uint8_t device_id;
};

int main()
{
    std::cout << "[canfd_txrx] Starting" << std::endl;

    // Set CPU affinity if specified
    const char* cpu_affinity_env = std::getenv("CPU_AFFINITY");
    if (cpu_affinity_env) {
        int cpu_id = std::atoi(cpu_affinity_env);
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_id, &cpuset);
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0) {
            std::cout << "[canfd_txrx] CPU affinity set to core " << cpu_id << std::endl;
        } else {
            std::cerr << "[canfd_txrx] Failed to set CPU affinity to core " << cpu_id << std::endl;
        }
    }

    // Set RT priority (SCHED_FIFO) if specified
    const char* rt_priority_env = std::getenv("RT_PRIORITY");
    if (rt_priority_env) {
        int priority = std::atoi(rt_priority_env);
        struct sched_param param;
        param.sched_priority = priority;
        if (sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
            std::cout << "[canfd_txrx] RT priority set to SCHED_FIFO " << priority << std::endl;
        } else {
            std::cerr << "[canfd_txrx] Failed to set RT priority (need CAP_SYS_NICE)" << std::endl;
        }
    }

    // Get CAN interface from env (default: can0)
    const char* can_if = std::getenv("CAN_INTERFACE");
    if (!can_if) can_if = "can0";

    // Get RX timeout from env (default: 1000us = 1ms)
    int rx_timeout_us = 1000;
    const char* timeout_str = std::getenv("RX_TIMEOUT_US");
    if (timeout_str) rx_timeout_us = std::atoi(timeout_str);

    // Initialize dora context
    auto dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        std::cerr << "[canfd_txrx] Failed to init dora context" << std::endl;
        return 1;
    }

    // Create CAN interface instance
    CanInterface can(can_if, rx_timeout_us);
    if (!can.init())
    {
        std::cerr << "[canfd_txrx] Failed to initialize CAN interface" << std::endl;
        free_dora_context(dora_context);
        return 1;
    }

    std::cout << "[canfd_txrx] Bound to " << can_if
              << ", RX timeout: " << rx_timeout_us << "us" << std::endl;
    std::cout << "[canfd_txrx] Group split: device_id <" << static_cast<int>(GROUP_BOUNDARY)
              << " (group1/右足), >=" << static_cast<int>(GROUP_BOUNDARY) << " (group2/左足)" << std::endl;
    std::cout << "[canfd_txrx] Waiting for axis_config..." << std::endl;

    while (true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            break;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Stop)
        {
            std::cout << "[canfd_txrx] Received stop signal" << std::endl;
            free_dora_event(event);
            break;
        }
        else if (ty == DoraEventType_Input)
        {
            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string input_id(id_ptr, id_len);

            char *data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);

            if (input_id == "axis_config")
            {
                // Parse axis config: [1 byte axis_count][device_id 1][device_id 2]...
                if (!g_config_received && data_len >= 1)
                {
                    uint8_t axis_count = static_cast<uint8_t>(data_ptr[0]);
                    g_device_ids.clear();

                    for (size_t i = 0; i < axis_count && (i + 1) < data_len; ++i)
                    {
                        g_device_ids.push_back(static_cast<uint8_t>(data_ptr[1 + i]));
                    }

                    g_config_received = true;
                    std::cout << "[canfd_txrx] Config received: " << g_device_ids.size() << " axes (device_ids: ";
                    for (size_t i = 0; i < g_device_ids.size(); ++i)
                    {
                        if (i > 0) std::cout << ", ";
                        std::cout << static_cast<int>(g_device_ids[i]);
                    }
                    std::cout << ")" << std::endl;
                }
            }
            else if (input_id == "can_frames")
            {
                // Multiple frames: [1 byte frame_count][frame1...][frame2...]
                // Each frame: [4 bytes arb_id][1 byte len][data...]
                if (g_config_received && data_len >= 1)
                {
                    uint8_t frame_count = static_cast<uint8_t>(data_ptr[0]);
                    size_t offset = 1;

                    // Record TX start time for cycle time measurement
                    auto tx_start_time = std::chrono::steady_clock::now();

                    // Parse all frames and split into two groups
                    std::vector<TxFrame> group1_frames;  // device_id 1-3
                    std::vector<TxFrame> group2_frames;  // device_id 4-6

                    for (uint8_t i = 0; i < frame_count && offset < data_len; ++i)
                    {
                        if (offset + 5 > data_len) break;

                        TxFrame frame;
                        std::memcpy(&frame.arb_id, data_ptr + offset, 4);
                        offset += 4;

                        frame.len = static_cast<uint8_t>(data_ptr[offset]);
                        offset += 1;

                        if (frame.len > 64) frame.len = 64;
                        if (offset + frame.len > data_len) break;

                        std::memcpy(frame.data, data_ptr + offset, frame.len);
                        offset += frame.len;

                        // Extract device_id from arb_id (lower 7 bits)
                        frame.device_id = frame.arb_id & 0x7F;

                        // Split into groups based on device_id
                        if (frame.device_id < GROUP_BOUNDARY)
                        {
                            group1_frames.push_back(frame);
                        }
                        else
                        {
                            group2_frames.push_back(frame);
                        }

                    }

                    // Collect all RX frames from both groups
                    std::vector<CanInterface::RxFrame> all_rx_frames;
                    int total_received = 0;

                    // === Group 1: Send and receive (右足 device_id 1-3) ===
                    if (!group1_frames.empty())
                    {
                        std::vector<uint32_t> expected_ids;
                        for (const auto& frame : group1_frames)
                        {
                            can.send(frame.arb_id, frame.data, frame.len);
                            expected_ids.push_back(static_cast<uint32_t>(frame.device_id) << 8);
                        }

                        std::vector<CanInterface::RxFrame> rx_frames;
                        int64_t rx_time_us = 0;
                        int received = can.receiveMultiple(expected_ids, rx_frames, rx_time_us);
                        total_received += received;

                        for (auto& rx : rx_frames)
                        {
                            if (rx.len > 0)
                            {
                                all_rx_frames.push_back(rx);
                            }
                        }
                    }

                    // === Group 2: Send and receive (左足 device_id 4-6) ===
                    if (!group2_frames.empty())
                    {
                        std::vector<uint32_t> expected_ids;
                        for (const auto& frame : group2_frames)
                        {
                            can.send(frame.arb_id, frame.data, frame.len);
                            expected_ids.push_back(static_cast<uint32_t>(frame.device_id) << 8);
                        }

                        std::vector<CanInterface::RxFrame> rx_frames;
                        int64_t rx_time_us = 0;
                        int received = can.receiveMultiple(expected_ids, rx_frames, rx_time_us);
                        total_received += received;

                        for (auto& rx : rx_frames)
                        {
                            if (rx.len > 0)
                            {
                                all_rx_frames.push_back(rx);
                            }
                        }
                    }

                    // Calculate total cycle time from TX start to RX complete
                    auto tx_end_time = std::chrono::steady_clock::now();
                    int64_t total_cycle_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        tx_end_time - tx_start_time).count();

                    if (total_received > 0)
                    {
                        // Build rx_frames output
                        // Format: [1 byte frame_count][rx_frame1...][rx_frame2...]
                        // Each rx_frame: [8 bytes timestamp][4 bytes arb_id][1 byte len][data...]
                        uint8_t rx_buffer[4096];
                        size_t rx_offset = 1;  // Reserve first byte for frame count

                        for (const auto& rx : all_rx_frames)
                        {
                            std::memcpy(rx_buffer + rx_offset, &rx.timestamp_ns, 8);
                            rx_offset += 8;

                            std::memcpy(rx_buffer + rx_offset, &rx.arb_id, 4);
                            rx_offset += 4;

                            rx_buffer[rx_offset] = rx.len;
                            rx_offset += 1;

                            std::memcpy(rx_buffer + rx_offset, rx.data, rx.len);
                            rx_offset += rx.len;
                        }

                        rx_buffer[0] = static_cast<uint8_t>(all_rx_frames.size());

                        // Output rx_frames
                        std::string output_id = "rx_frames";
                        dora_send_output(dora_context,
                                         const_cast<char*>(output_id.c_str()), output_id.length(),
                                         reinterpret_cast<char*>(rx_buffer), rx_offset);

                        // Output total cycle time (TX start to RX complete, including timeouts)
                        output_id = "cycle_time_us";
                        dora_send_output(dora_context,
                                         const_cast<char*>(output_id.c_str()), output_id.length(),
                                         reinterpret_cast<char*>(&total_cycle_time_us), sizeof(total_cycle_time_us));
                    }
                }
            }
        }

        free_dora_event(event);
    }

    free_dora_context(dora_context);
    std::cout << "[canfd_txrx] Finished" << std::endl;
    return 0;
}
