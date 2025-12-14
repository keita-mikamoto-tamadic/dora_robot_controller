#include "timing_manager.hpp"

TimingManager::TimingManager()
    : m_elapsed_time(0.0)
    , m_last_delta(0.0)
    , m_initialized(false)
{
}

double TimingManager::update()
{
    auto now = std::chrono::steady_clock::now();

    if (!m_initialized) {
        m_last_tick = now;
        m_start_time = now;
        m_initialized = true;
        m_last_delta = 0.0;
        return 0.0;
    }

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - m_last_tick);
    m_last_delta = duration.count() / 1000000.0;  // Convert to seconds
    m_last_tick = now;

    // Update elapsed time from start
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - m_start_time);
    m_elapsed_time = elapsed.count() / 1000000.0;

    return m_last_delta;
}

void TimingManager::reset()
{
    m_start_time = std::chrono::steady_clock::now();
    m_last_tick = m_start_time;
    m_elapsed_time = 0.0;
    m_last_delta = 0.0;
    m_initialized = true;
}

double TimingManager::getElapsedTime() const
{
    return m_elapsed_time;
}

double TimingManager::getLastDeltaTime() const
{
    return m_last_delta;
}
