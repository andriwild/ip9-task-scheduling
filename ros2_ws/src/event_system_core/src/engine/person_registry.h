#pragma once

#include <map>
#include <optional>
#include <random>
#include <string>

#include "util/types.h"

class PersonRegistry {
    des::PersonList m_people;
    des::PersonMap m_employees;
    std::map<std::string, std::string> m_roomByPerson;
    std::map<std::string, des::Point> m_positionByPerson;
    const des::RoomMap& m_rooms;
    std::mt19937 m_placementRng;

public:
    static constexpr unsigned int PLACEMENT_SEED = 1337;

    PersonRegistry(
        des::PersonList people,
        const des::RoomMap& rooms,
        unsigned int seed = PLACEMENT_SEED
    );

    void reseed(unsigned int seed);
    void reseedPersons(unsigned int seed);

    bool hasEmployee(const std::string& name) const;
    des::Person* getByName(const std::string& name) const;
    const des::PersonList& all() const;

    std::string room(const std::string& person) const;
    const std::map<std::string, std::string>& allRooms() const;
    void setRoom(const std::string& person, const std::string& room);
    std::optional<des::Point> position(const std::string& person) const;
    bool isAt(const std::string& person, const std::string& room) const;

    void clearRooms();

private:
    std::optional<des::Point> samplePosition(const std::string& room);
};
