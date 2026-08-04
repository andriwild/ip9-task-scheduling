/*
 * Class to store last service time of a job.
 * Used for prioritizing recurrent missions.
 * 
 */
#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>

class ServiceLog {
    std::map<std::pair<std::string, std::string>, int> m_lastServiced;

public:
    std::optional<int> lastServiced(const std::string& room, const std::string& type) const;
    void recordServiced(const std::string& room, const std::string& type, int time);
    void clear();
};
