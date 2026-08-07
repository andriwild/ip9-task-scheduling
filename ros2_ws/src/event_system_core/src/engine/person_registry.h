/*
 * Owns the simulated people and their locations.
 * Maps name to Person, tracks the current room per person and
 * samples a concrete point inside that room's footprint.
 * Placement uses its own RNG stream. 
 * 
 */

#pragma once

#include <map>
#include <optional>
#include <random>
#include <string>

#include "util/types.h"


namespace des {

class PersonRegistry {
    PersonList m_people;
    PersonMap m_employees;
    std::map<std::string, std::string> m_roomByPerson;
    std::map<std::string, Point> m_positionByPerson;
    const RoomMap& m_rooms;
    std::mt19937 m_placementRng;

public:
    static constexpr unsigned int PLACEMENT_SEED = 1337;

    PersonRegistry(
        PersonList people,
        const RoomMap& rooms,
        unsigned int seed = PLACEMENT_SEED
    );

    void reseed(unsigned int seed);
    void reseedPersons(unsigned int seed);

    bool hasEmployee(const std::string& name) const;
    Person* getByName(const std::string& name) const;
    const PersonList& all() const;

    std::string room(const std::string& person) const;
    const std::map<std::string, std::string>& allRooms() const;
    void setRoom(const std::string& person, const std::string& room);
    std::optional<Point> position(const std::string& person) const;
    bool isAt(const std::string& person, const std::string& room) const;

    void clearRooms();

private:
    std::optional<Point> samplePosition(const std::string& room);
};

}  // namespace des
