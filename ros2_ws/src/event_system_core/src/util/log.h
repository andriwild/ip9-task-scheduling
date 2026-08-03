#pragma once

#include <atomic>
#include <cstdlib>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "types.h"

namespace des::log {

inline std::atomic<int>  g_sim_time{0};
inline std::atomic<bool> g_time_set{false};

inline void setSimTime(int t) noexcept {
    g_sim_time.store(t, std::memory_order_relaxed);
    g_time_set.store(true, std::memory_order_relaxed);
}

inline std::string nowStr() {
    if (!g_time_set.load(std::memory_order_relaxed)) {
        return "--:--:--";
    }
    return ::des::toHumanReadableTime(g_sim_time.load(std::memory_order_relaxed));
}

// Defaults for the built-in rcutils console handler. Must run before rclcpp::init().
// overwrite=0 keeps whatever the environment already provides.
inline void setConsoleDefaults() {
    setenv("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}", 0);
    setenv("RCUTILS_LOGGING_USE_STDOUT", "1", 0);
    setenv("RCUTILS_LOGGING_BUFFERED_STREAM", "0", 0);
}

}  // namespace des::log

#define DES_LOG_INFO(logger, fmt, ...) \
    RCLCPP_INFO((logger), "[%s] " fmt, ::des::log::nowStr().c_str() __VA_OPT__(, ) __VA_ARGS__)

#define DES_LOG_DEBUG(logger, fmt, ...) \
    RCLCPP_DEBUG((logger), "[%s] " fmt, ::des::log::nowStr().c_str() __VA_OPT__(, ) __VA_ARGS__)

#define DES_LOG_WARN(logger, fmt, ...) \
    RCLCPP_WARN((logger), "[%s] " fmt, ::des::log::nowStr().c_str() __VA_OPT__(, ) __VA_ARGS__)

#define DES_LOG_ERROR(logger, fmt, ...) \
    RCLCPP_ERROR((logger), "[%s] " fmt, ::des::log::nowStr().c_str() __VA_OPT__(, ) __VA_ARGS__)

#define DES_LOG_INFO_STREAM(logger, stream_args) \
    RCLCPP_INFO_STREAM((logger), "[" << ::des::log::nowStr() << "] " << stream_args)

#define DES_LOG_DEBUG_STREAM(logger, stream_args) \
    RCLCPP_DEBUG_STREAM((logger), "[" << ::des::log::nowStr() << "] " << stream_args)
