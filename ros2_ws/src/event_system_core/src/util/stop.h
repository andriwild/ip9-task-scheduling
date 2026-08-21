/*
 * Ctrl-C handling for the simulation loop.
 * Replaces rclcpp::ok() as the loop condition so the loop does not depend on ROS being initialized.
 *
 */

#pragma once

#include <atomic>
#include <csignal>

namespace des::stop {

inline std::atomic<bool> g_stop{false};

inline void request(int) noexcept {
    g_stop.store(true, std::memory_order_relaxed);
}

inline void installHandlers() {
    std::signal(SIGINT, request);
    std::signal(SIGTERM, request);
}

inline bool requested() noexcept {
    return g_stop.load(std::memory_order_relaxed);
}

}  // namespace des::stop
