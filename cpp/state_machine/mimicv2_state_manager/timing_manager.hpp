#pragma once

#include <chrono>

class TimingManager {
public:
    TimingManager();

    // Call on each tick, returns delta time in seconds since last call
    double update();

    // Reset elapsed time (call when starting interpolation)
    void reset();

    // Get total elapsed time since reset (seconds)
    double getElapsedTime() const;

    // Get last delta time (seconds)
    double getLastDeltaTime() const;

private:
    std::chrono::steady_clock::time_point m_last_tick;
    std::chrono::steady_clock::time_point m_start_time;
    double m_elapsed_time;
    double m_last_delta;
    bool m_initialized;
};
