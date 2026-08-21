// Generated with Claude Code (Anthropic), then reviewed and adapted by the author. See the index of auxiliary tools.

#pragma once

#include <atomic>
#include <cstdio>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include "types.h"

namespace des::log {

enum class Level { Debug, Info, Warn, Error };

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

inline std::map<std::string, Level>& levels() {
    static std::map<std::string, Level> configured{{"", Level::Info}};
    return configured;
}

// A level set on "des.bt" applies to "des.bt.charge" as well, the longest prefix wins.
inline void setLevel(std::string prefix, const Level level) {
    levels()[std::move(prefix)] = level;
}

inline Level levelOf(const char* name) {
    const std::string_view full(name);
    Level best = Level::Info;
    std::size_t longest = 0;
    for (const auto& [prefix, level] : levels()) {
        if (full.starts_with(prefix) && prefix.size() >= longest) {
            best    = level;
            longest = prefix.size();
        }
    }
    return best;
}

inline std::optional<Level> levelFromString(const std::string_view text) {
    if (text == "DEBUG") { return Level::Debug; }
    if (text == "INFO")  { return Level::Info; }
    if (text == "WARN")  { return Level::Warn; }
    if (text == "ERROR") { return Level::Error; }
    return std::nullopt;
}

inline const char* tag(const Level level) {
    switch (level) {
        case Level::Debug: return "DEBUG";
        case Level::Info:  return "INFO";
        case Level::Warn:  return "WARN";
        case Level::Error: return "ERROR";
    }
    return "INFO";
}

inline void write(const Level level, const char* name, const char* message) {
    std::printf("[%s] [%s]: [%s] %s\n", tag(level), name, nowStr().c_str(), message);
}

// Reads "--log-level des.bt:=DEBUG" style arguments, the same form ros2 launch passes on.
inline void applyArgs(const int argc, char* argv[]) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string_view(argv[i]) != "--log-level") {
            continue;
        }
        const std::string_view spec(argv[i + 1]);
        const auto split = spec.find(":=");
        if (split == std::string_view::npos) {
            continue;
        }
        if (const auto level = levelFromString(spec.substr(split + 2))) {
            setLevel(std::string(spec.substr(0, split)), *level);
        }
    }
}

}  // namespace des::log

#define DES_LOG(level, name, fmt, ...)                                          \
    do {                                                                        \
        if ((level) >= ::des::log::levelOf(name)) {                             \
            char buffer[1024];                                                  \
            std::snprintf(buffer, sizeof(buffer), fmt __VA_OPT__(, ) __VA_ARGS__); \
            ::des::log::write((level), (name), buffer);                         \
        }                                                                       \
    } while (0)

#define DES_LOG_DEBUG(name, fmt, ...) DES_LOG(::des::log::Level::Debug, name, fmt __VA_OPT__(, ) __VA_ARGS__)
#define DES_LOG_INFO(name, fmt, ...)  DES_LOG(::des::log::Level::Info,  name, fmt __VA_OPT__(, ) __VA_ARGS__)
#define DES_LOG_WARN(name, fmt, ...)  DES_LOG(::des::log::Level::Warn,  name, fmt __VA_OPT__(, ) __VA_ARGS__)
#define DES_LOG_ERROR(name, fmt, ...) DES_LOG(::des::log::Level::Error, name, fmt __VA_OPT__(, ) __VA_ARGS__)

#define DES_LOG_DEBUG_STREAM(name, stream_args)                                 \
    do {                                                                        \
        if (::des::log::Level::Debug >= ::des::log::levelOf(name)) {            \
            std::ostringstream oss;                                             \
            oss << stream_args;                                                 \
            ::des::log::write(::des::log::Level::Debug, (name), oss.str().c_str()); \
        }                                                                       \
    } while (0)
