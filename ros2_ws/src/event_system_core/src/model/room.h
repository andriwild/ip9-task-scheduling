#pragma once

#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "../util/point.h"

namespace des {

enum class RoomType {
    OFFICE,
    CLASSROOM,
    MEETING,
    KITCHEN,
    TOILET,
    ACCESS,
    SPACE,
    MISC
};

inline std::optional<RoomType> roomTypeFromString(const std::string& type) {
    if (type == "OFFICE") {
        return RoomType::OFFICE;
    }
    if (type == "CLASSROOM") {
        return RoomType::CLASSROOM;
    }
    if (type == "MEETING") {
        return RoomType::MEETING;
    }
    if (type == "KITCHEN") {
        return RoomType::KITCHEN;
    }
    if (type == "TOILET") {
        return RoomType::TOILET;
    }
    if (type == "ACCESS") {
        return RoomType::ACCESS;
    }
    if (type == "SPACE") {
        return RoomType::SPACE;
    }
    if (type == "MISC") {
        return RoomType::MISC;
    }
    return std::nullopt;
}

inline std::string roomTypeToString(const RoomType type) {
    switch (type) {
        case RoomType::OFFICE:    return "OFFICE";
        case RoomType::CLASSROOM: return "CLASSROOM";
        case RoomType::MEETING:   return "MEETING";
        case RoomType::KITCHEN:   return "KITCHEN";
        case RoomType::TOILET:    return "TOILET";
        case RoomType::ACCESS:    return "ACCESS";
        case RoomType::SPACE:     return "SPACE";
        case RoomType::MISC:      return "MISC";
    }
    return "MISC";
}

struct RoomTour {
    double m_distance = 0.0;
    std::vector<Point> m_path;
    std::vector<Polygon> m_visPolys;

    bool empty() const {
        return m_path.empty();
    }

    const Polygon& visibilityAt(const std::size_t index) const {
        static const Polygon unbounded;
        if (index >= m_visPolys.size()) {
            return unbounded;
        }
        return m_visPolys[index];
    }
};

struct Room {
    std::string m_name;
    Point m_waypoint;
    std::optional<double> m_area;
    std::vector<Point> m_footprint;
    RoomType m_roomType = RoomType::MISC;
    RoomTour m_tour;

    explicit Room(const std::string& name, const Point& waypoint, const std::optional<double> area = std::nullopt)
    : m_name(name)
    , m_waypoint(waypoint)
    , m_area(area)
    {}

    friend std::ostream& operator<<(std::ostream& os, const Room& r) {
        os << r.m_name << r.m_waypoint;
        if (r.m_area) os << " area=" << *r.m_area;
        os << " type=" << roomTypeToString(r.m_roomType);
        if (!r.m_tour.empty()) os << " tour=" << r.m_tour.m_distance << "m/" << r.m_tour.m_path.size() << "pts";
        return os;
    }
};

using RoomMap = std::map<std::string, Room>;

}  // namespace des
