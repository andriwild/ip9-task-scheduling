#include "person_registry.h"

#include <utility>

#include "../util/geometry.h"

PersonRegistry::PersonRegistry(
    des::PersonMap employees,
    const des::LocationMap& locationMap
)
    : m_employees(std::move(employees))
    , m_locationMap(locationMap)
    , m_placementRng(PLACEMENT_SEED)
{
}

bool PersonRegistry::hasEmployee(const std::string& name) const {
    return m_employees.contains(name);
}

des::Person* PersonRegistry::getByName(const std::string& name) const {
    return m_employees.at(name);
}

std::string PersonRegistry::location(const std::string& name) const {
    return m_locations.at(name);
}

const std::map<std::string, std::string>& PersonRegistry::allLocations() const {
    return m_locations;
}

void PersonRegistry::setLocation(const std::string& name, const std::string& room) {
    m_locations[name] = room;
    if (const auto pos = samplePosition(room)) {
        m_positions[name] = *pos;
    } else {
        m_positions.erase(name);
    }
}

std::optional<des::Point> PersonRegistry::position(const std::string& name) const {
    const auto it = m_positions.find(name);
    if (it == m_positions.end()) {
        return std::nullopt;
    }
    return it->second;
}

bool PersonRegistry::isAt(const std::string& name, const std::string& location) const {
    const auto it = m_locations.find(name);
    if (it == m_locations.end()) {
        return false;
    }
    return it->second == location;
}

void PersonRegistry::clearLocations() {
    m_locations.clear();
}

std::optional<des::Point> PersonRegistry::samplePosition(const std::string& room) {
    const auto it = m_locationMap.find(room);
    if (it == m_locationMap.end()) {
        return std::nullopt;
    }
    if (const auto pos = geom::sampleInPolygon(it->second.m_footprint, m_placementRng)) {
        return pos;
    }
    return it->second.m_p;
}
