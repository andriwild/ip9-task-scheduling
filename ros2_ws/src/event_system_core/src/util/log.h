#pragma once

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include "types.h"

namespace des::log {

inline std::atomic<int>  g_sim_time{0};
inline std::atomic<bool> g_time_set{false};
inline bool              g_colorize{false};

inline void setSimTime(int t) noexcept {
    g_sim_time.store(t, std::memory_order_relaxed);
    g_time_set.store(true, std::memory_order_relaxed);
}

inline void resetSimTime() noexcept {
    g_sim_time.store(0, std::memory_order_relaxed);
    g_time_set.store(false, std::memory_order_relaxed);
}

inline std::string nowStr() {
    if (!g_time_set.load(std::memory_order_relaxed)) {
        return "--:--:--";
    }
    return ::des::toHumanReadableTime(g_sim_time.load(std::memory_order_relaxed));
}

namespace detail {

// 5-char fixed-width labels keep downstream columns aligned.
inline const char* sevLabel(int severity) {
    switch (severity) {
        case RCUTILS_LOG_SEVERITY_DEBUG: return "DEBUG";
        case RCUTILS_LOG_SEVERITY_INFO:  return "INFO ";
        case RCUTILS_LOG_SEVERITY_WARN:  return "WARN ";
        case RCUTILS_LOG_SEVERITY_ERROR: return "ERROR";
        case RCUTILS_LOG_SEVERITY_FATAL: return "FATAL";
        default:                         return "?????";
    }
}

inline const char* sevColor(int severity) {
    if (!g_colorize) return "";
    switch (severity) {
        case RCUTILS_LOG_SEVERITY_DEBUG: return "\033[37m";    // gray
        case RCUTILS_LOG_SEVERITY_INFO:  return "\033[1;32m";  // bold green
        case RCUTILS_LOG_SEVERITY_WARN:  return "\033[33m";    // yellow
        case RCUTILS_LOG_SEVERITY_ERROR: return "\033[31m";    // red
        case RCUTILS_LOG_SEVERITY_FATAL: return "\033[1;31m";  // bold red
        default:                         return "";
    }
}

inline void desOutputHandler(
    const rcutils_log_location_t* /*location*/,
    int severity,
    const char* /*name*/,
    rcutils_time_point_value_t /*timestamp*/,
    const char* format,
    va_list* args)
{
    char user_msg[2048];
    va_list args_copy;
    va_copy(args_copy, *args);
    std::vsnprintf(user_msg, sizeof(user_msg), format, args_copy);
    va_end(args_copy);

    const char* color = sevColor(severity);
    const char* reset = (color[0] != '\0') ? "\033[0m" : "";

    // single stream + flush — stdout/stderr buffer independently under `ros2 launch`,
    // which would otherwise re-order lines on merge.
    std::fprintf(stderr, "%s[%s]%s %s\n", color, sevLabel(severity), reset, user_msg);
    std::fflush(stderr);
}

}  // namespace detail

// Call once after rclcpp::init().
inline void installOutputHandler() {
    const char* env = std::getenv("RCUTILS_COLORIZED_OUTPUT");
    if (env && env[0] != '\0') {
        g_colorize = (std::string(env) == "1");
    } else {
        g_colorize = isatty(fileno(stdout)) != 0;
    }
    rcutils_logging_set_output_handler(detail::desOutputHandler);
}

}  // namespace des::log

#define DES_LOG_INFO(logger, fmt, ...)                                                    \
    do {                                                                                  \
        auto _des_lg = (logger);                                                          \
        RCLCPP_INFO(_des_lg, "[%s] [%s]: " fmt, ::des::log::nowStr().c_str(),             \
                    _des_lg.get_name() __VA_OPT__(, ) __VA_ARGS__);                       \
    } while (0)

#define DES_LOG_DEBUG(logger, fmt, ...)                                                   \
    do {                                                                                  \
        auto _des_lg = (logger);                                                          \
        RCLCPP_DEBUG(_des_lg, "[%s] [%s]: " fmt, ::des::log::nowStr().c_str(),            \
                     _des_lg.get_name() __VA_OPT__(, ) __VA_ARGS__);                      \
    } while (0)

#define DES_LOG_WARN(logger, fmt, ...)                                                    \
    do {                                                                                  \
        auto _des_lg = (logger);                                                          \
        RCLCPP_WARN(_des_lg, "[%s] [%s]: " fmt, ::des::log::nowStr().c_str(),             \
                    _des_lg.get_name() __VA_OPT__(, ) __VA_ARGS__);                       \
    } while (0)

#define DES_LOG_ERROR(logger, fmt, ...)                                                   \
    do {                                                                                  \
        auto _des_lg = (logger);                                                          \
        RCLCPP_ERROR(_des_lg, "[%s] [%s]: " fmt, ::des::log::nowStr().c_str(),            \
                     _des_lg.get_name() __VA_OPT__(, ) __VA_ARGS__);                      \
    } while (0)

#define DES_LOG_INFO_STREAM(logger, stream_args)                                          \
    do {                                                                                  \
        auto _des_lg = (logger);                                                          \
        RCLCPP_INFO_STREAM(_des_lg, "[" << ::des::log::nowStr() << "] ["                  \
                                       << _des_lg.get_name() << "]: " << stream_args);   \
    } while (0)

#define DES_LOG_DEBUG_STREAM(logger, stream_args)                                         \
    do {                                                                                  \
        auto _des_lg = (logger);                                                          \
        RCLCPP_DEBUG_STREAM(_des_lg, "[" << ::des::log::nowStr() << "] ["                 \
                                        << _des_lg.get_name() << "]: " << stream_args);  \
    } while (0)
