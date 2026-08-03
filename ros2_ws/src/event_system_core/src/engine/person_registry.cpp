#include "engine/person_registry.h"

#include <utility>

#include "util/geometry.h"

PersonRegistry::PersonRegistry(
    des::PersonList people,
    const des::RoomMap& rooms,
    const unsigned int seed
)
    : m_people(std::move(people))
    , m_rooms(rooms)
    , m_placementRng(seed)
{
    for (const auto& person : m_people) {
        m_employees[person->firstName] = person.get();
    }
}

const des::PersonList& PersonRegistry::all() const {
    return m_people;
}

void PersonRegistry::reseed(const unsigned int seed) {
    m_placementRng.seed(seed);
}

void PersonRegistry::reseedPersons(const unsigned int seed) {
    for (const auto& [name, person] : m_employees) {
        person->reseed(seed);
    }
}

bool PersonRegistry::hasEmployee(const std::string& name) const {
    return m_employees.contains(name);
}

des::Person* PersonRegistry::getByName(const std::string& name) const {
    return m_employees.at(name);
}

std::string PersonRegistry::room(const std::string& person) const {
    return m_roomByPerson.at(person);
}

const std::map<std::string, std::string>& PersonRegistry::allRooms() const {
    return m_roomByPerson;
}

void PersonRegistry::setRoom(const std::string& person, const std::string& room) {
    m_roomByPerson[person] = room;
    if (const auto pos = samplePosition(room)) {
        m_positionByPerson[person] = *pos;
    } else {
        m_positionByPerson.erase(person);
    }
}

std::optional<des::Point> PersonRegistry::position(const std::string& person) const {
    const auto it = m_positionByPerson.find(person);
    if (it == m_positionByPerson.end()) {
        return std::nullopt;
    }
    return it->second;
}

bool PersonRegistry::isAt(const std::string& person, const std::string& room) const {
    const auto it = m_roomByPerson.find(person);
    if (it == m_roomByPerson.end()) {
        return false;
    }
    return it->second == room;
}

void PersonRegistry::clearRooms() {
    m_roomByPerson.clear();
}

std::optional<des::Point> PersonRegistry::samplePosition(const std::string& room) {
    const auto it = m_rooms.find(room);
    if (it == m_rooms.end()) {
        return std::nullopt;
    }
    if (const auto pos = geom::sampleInPolygon(it->second.m_footprint, m_placementRng)) {
        return pos;
    }
    return it->second.m_p;
}
