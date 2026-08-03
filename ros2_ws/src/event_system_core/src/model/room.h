#pragma once

#include <map>
#include <optional>
#include <ostream>
#include <regex>
#include <string>
#include <vector>

#include "../util/point.h"

namespace des {

enum class RoomType {
    WORKPLACE,
    CLASSROOM,
    TOILET,
    KITCHEN,
    OTHER
};

inline RoomType roomTypeFromString(const std::string& type) {
    if (type == "WORKPLACE") {
        return RoomType::WORKPLACE;
    }
    if (type == "CLASSROOM") {
        return RoomType::CLASSROOM;
    }
    if (type == "TOILET") {
        return RoomType::TOILET;
    }
    if (type == "KITCHEN") {
        return RoomType::KITCHEN;
    }
    return RoomType::OTHER;
}

inline std::string roomTypeToString(const RoomType type) {
    switch (type) {
        case RoomType::WORKPLACE: return "WORKPLACE";
        case RoomType::CLASSROOM: return "CLASSROOM";
        case RoomType::TOILET:    return "TOILET";
        case RoomType::KITCHEN:   return "KITCHEN";
        case RoomType::OTHER:     return "OTHER";
    }
    return "OTHER";
}

inline RoomType parseRoomName(const std::string& roomName) {
    if (roomName.find("Toilet") != std::string::npos) return RoomType::TOILET;
    if (roomName.find("Kitchen") != std::string::npos) return RoomType::KITCHEN;
    static const std::regex workplacePattern(R"(5\.2[A-Z]\d+)");
    if (std::regex_match(roomName, workplacePattern)) return RoomType::WORKPLACE;
    return RoomType::OTHER;
}

struct RoomTour {
    double m_distance = 0.0;
    int m_steps = 0;
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
    Point m_p;
    std::optional<double> m_area;
    std::vector<Point> m_footprint;
    RoomType m_roomType = RoomType::OTHER;
    RoomTour m_tour;

    explicit Room(const std::string& name, const Point p, const std::optional<double> area = std::nullopt) : m_name(name), m_p(p), m_area(area) {}

    friend std::ostream& operator<<(std::ostream& os, const Room& r) {
        os << r.m_name << r.m_p;
        if (r.m_area) os << " area=" << *r.m_area;
        os << " type=" << roomTypeToString(r.m_roomType);
        if (!r.m_tour.empty()) os << " tour=" << r.m_tour.m_distance << "m/" << r.m_tour.m_path.size() << "pts";
        return os;
    }
};

using RoomMap = std::map<std::string, Room>;

}  // namespace des
