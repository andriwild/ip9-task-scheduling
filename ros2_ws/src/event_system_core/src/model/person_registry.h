#pragma once

#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>

#include "../util/types.h"

class PersonRegistry {
    des::PersonMap m_employees;
    std::map<std::string, std::string> m_locations;
    std::map<std::string, des::Point> m_positions;
    const des::LocationMap& m_locationMap;
    std::mt19937 m_placementRng;

public:
    static constexpr unsigned int PLACEMENT_SEED = 1337;

    PersonRegistry(
        des::PersonMap employees,
        const des::LocationMap& locationMap
    );

    bool hasEmployee(const std::string& name) const;
    des::Person* getByName(const std::string& name) const;

    std::string location(const std::string& name) const;
    const std::map<std::string, std::string>& allLocations() const;
    void setLocation(const std::string& name, const std::string& room);
    std::optional<des::Point> position(const std::string& name) const;
    bool isAt(const std::string& name, const std::string& location) const;

    void clearLocations();

private:
    std::optional<des::Point> samplePosition(const std::string& room);
};
