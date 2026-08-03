#include "engine/service_log.h"

std::optional<int> ServiceLog::lastServiced(const std::string& room, const std::string& type) const {
    const auto it = m_lastServiced.find({room, type});
    if (it == m_lastServiced.end()) {
        return std::nullopt;
    }
    return it->second;
}

void ServiceLog::recordServiced(const std::string& room, const std::string& type, const int time) {
    m_lastServiced[{room, type}] = time;
}

void ServiceLog::clear() {
    m_lastServiced.clear();
}
