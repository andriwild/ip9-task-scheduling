#pragma once

#include <nlohmann/json.hpp>

#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "tsp.h"

namespace TourIO {

const std::string kConfigDir = "config";
const std::string kBuildingFile = kConfigDir + "/building.json";

inline std::string toursPath(const double radius) {
    std::ostringstream oss;
    oss << kConfigDir << "/tours/tours_r" << radius << ".json";
    return oss.str();
}

inline std::vector<TSP::Room> readBuilding(const std::string& path) {
    std::ifstream file(path);
    assert(file.is_open());
    const nlohmann::json json = nlohmann::json::parse(file);

    std::vector<TSP::Room> rooms;
    for (const auto& entry : json.at("rooms")) {
        TSP::Room room;
        room.m_name = entry.at("name").get<std::string>();
        room.m_waypoint = TSP::Vec2{entry.at("x").get<double>(), entry.at("y").get<double>()};
        if (entry.contains("footprint")) {
            for (const auto& vertex : entry.at("footprint")) {
                room.m_footprint.push_back(TSP::Vec2{vertex.at(0).get<double>(), vertex.at(1).get<double>()});
            }
        }
        rooms.push_back(std::move(room));
    }
    return rooms;
}

inline void writeTours(const std::string& path, const double radius, const std::vector<TSP::RoomTour>& tours) {
    nlohmann::ordered_json rooms = nlohmann::ordered_json::object();
    for (const TSP::RoomTour& tour : tours) {
        nlohmann::ordered_json entry;
        entry["steps"] = tour.m_steps;
        entry["distance"] = tour.m_distance;
        entry["start"] = {tour.m_start.m_x, tour.m_start.m_y};
        nlohmann::ordered_json path_ = nlohmann::ordered_json::array();
        for (const TSP::Vec2& point : tour.m_path) {
            path_.push_back({point.m_x, point.m_y});
        }
        entry["path"] = path_;
        if (!tour.m_visPolys.empty()) {
            nlohmann::ordered_json vis = nlohmann::ordered_json::array();
            for (const std::vector<TSP::Vec2>& polygon : tour.m_visPolys) {
                nlohmann::ordered_json ring = nlohmann::ordered_json::array();
                for (const TSP::Vec2& point : polygon) {
                    ring.push_back({point.m_x, point.m_y});
                }
                vis.push_back(ring);
            }
            entry["vis"] = vis;
        }
        rooms[tour.m_roomName] = entry;
    }

    nlohmann::ordered_json out;
    out["radius"] = radius;
    out["rooms"] = rooms;

    std::ofstream file(path);
    assert(file.is_open());
    file << out.dump(2) << "\n";
}

}  // namespace TourIO
