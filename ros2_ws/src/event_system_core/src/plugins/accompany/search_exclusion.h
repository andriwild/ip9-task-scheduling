#pragma once

#include <algorithm>
#include <string>
#include <vector>

inline bool isSearchExcluded(const std::vector<std::string>& excluded, const std::string& room) {
    return std::any_of(excluded.begin(), excluded.end(), [&](const std::string& pattern) {
        return room.find(pattern) != std::string::npos;
    });
}
