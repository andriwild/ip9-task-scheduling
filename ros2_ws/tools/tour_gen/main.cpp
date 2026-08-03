#include "VisLibRational.h"
#include "SightseeingTour1.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cassert>
#include <cmath>
#include <random>
#include <utility>

#include "tsp.h"

const std::string kConfigDir = "config";
const std::string kBuildingFile = kConfigDir + "/building.json";

std::string toursFile(double radius, const std::string& suffix) {
    std::ostringstream oss;
    oss << kConfigDir << "/tours_r" << radius << suffix << ".json";
    return oss.str();
}

std::string timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local);
    return std::string(buffer);
}

std::vector<TSP::Room> readBuilding(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("building snapshot not found: " + path + " (run ./build_snapshot.sh first)");
    }
    const nlohmann::json json = nlohmann::json::parse(file);

    const auto& names = json.at("names");
    const auto& footprints = json.at("footprints");
    const auto& locations = json.at("locations");

    std::vector<TSP::Room> rooms;
    rooms.reserve(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
        TSP::Room room;
        room.m_name = names.at(i).get<std::string>();
        room.m_waypoint = TSP::Vec2{locations.at(i).at("x").get<double>(), locations.at(i).at("y").get<double>()};
        if (i < footprints.size() && footprints.at(i).is_array()) {
            for (const auto& vertex : footprints.at(i)) {
                room.m_footprint.push_back(TSP::Vec2{vertex.at(0).get<double>(), vertex.at(1).get<double>()});
            }
        }
        rooms.push_back(std::move(room));
    }
    return rooms;
}

std::vector<VisLib::Point<VisLib::CT>> toExactPolygon(const std::vector<TSP::Vec2>& polygon) {
    std::vector<VisLib::Point<VisLib::CT>> points;
    points.reserve(polygon.size());
    for (const TSP::Vec2& vertex : polygon) {
        const VisLib::Point<VisLib::CT> point(VisLib::CT(vertex.m_x), VisLib::CT(vertex.m_y));
        if (!points.empty() && points.back() == point) {
            continue;
        }
        points.push_back(point);
    }
    if (points.size() >= 2 && points.front() == points.back()) {
        points.pop_back();
    }
    return points;
}

TSP::RoomTour computeTour(const std::vector<TSP::Vec2>& polygon, const TSP::Vec2& start, double radius) {
    TSP::RoomTour tour;
    const std::vector<VisLib::Point<VisLib::CT>> points = toExactPolygon(polygon);
    if (points.size() < 3) {
        tour.m_reason = "degenerate";
        return tour;
    }

    try {
        VisLib::SimplePolygon<VisLib::CT> simplePolygon(points);
        VisLib::DCEL<VisLib::CT> dcel(simplePolygon);
        const VisLib::Point<VisLib::CT> startPoint(VisLib::CT(start.m_x), VisLib::CT(start.m_y));
        const auto result = dcel.sightseeingTour1(startPoint, radius, VisLib::Show::Nothing);
        if (result.m_path.empty()) {
            tour.m_reason = "degenerate";
            return tour;
        }
        tour.m_ok = true;
        tour.m_start = TSP::Vec2{start.m_x, start.m_y, startPoint};
        tour.m_steps = result.m_path.size();
        tour.m_distance = static_cast<double>(result.m_distance);
        for (const auto& point : result.m_path) {
            tour.m_path.push_back(TSP::Vec2{static_cast<double>(point.m_x), static_cast<double>(point.m_y), point});
        }
    } catch (const std::exception&) {
        tour.m_reason = "throw";
    }
    return tour;
}

void addVisibility(TSP::RoomTour& tour, const std::vector<TSP::Vec2>& footprint) {
    tour.m_visPolys.clear();
    if (!tour.m_ok) {
        return;
    }
    const VisLib::SimplePolygon<VisLib::CT> room(toExactPolygon(footprint));
    tour.m_visPolys.reserve(tour.m_path.size());
    for (const TSP::Vec2& point : tour.m_path) {
        const VisLib::SimplePolygon<VisLib::CT> visibility = room.visibility(point.m_exact);
        std::vector<TSP::Vec2> ring;
        ring.reserve(visibility.size());
        for (const VisLib::Point<VisLib::CT>& vertex : visibility.m_polygon) {
            ring.push_back(TSP::Vec2{static_cast<double>(vertex.m_x), static_cast<double>(vertex.m_y), vertex});
        }
        tour.m_visPolys.push_back(std::move(ring));
    }
}

TSP::RoomTour computeRoomTour(const TSP::Room& room, double radius) {
    TSP::RoomTour tour;
    tour.m_roomName = room.m_name;
    if (room.m_footprint.size() < 3) {
        tour.m_reason = "no-footprint";
        return tour;
    }

    TSP::RoomTour result = computeTour(room.m_footprint, room.m_waypoint, radius);
    result.m_roomName = room.m_name;
    return result;
}

void writeTours(const std::string& path, double radius, const std::vector<TSP::RoomTour>& tours) {
    nlohmann::ordered_json rooms = nlohmann::ordered_json::object();
    for (const TSP::RoomTour& tour : tours) {
        nlohmann::ordered_json entry;
        entry["ok"] = tour.m_ok;
        if (!tour.m_ok) {
            entry["reason"] = tour.m_reason;
            rooms[tour.m_roomName] = entry;
            continue;
        }
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
    out["generated_at"] = timestamp();
    out["rooms"] = rooms;

    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("cannot write " + path);
    }
    file << out.dump(2) << "\n";
}


int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: tour_gen <radius>   (run from the workspace root)\n";
        return 2;
    }
    const double radius = std::stod(argv[1]);
    const std::string rawPath = toursFile(radius, "");
    const std::string tspPath = toursFile(radius, "_tsp");

    try {
        const std::vector<TSP::Room> rooms = readBuilding(kBuildingFile);
        std::cout << "read " << rooms.size() << " rooms from " << kBuildingFile << "\n";

        std::vector<TSP::RoomTour> rawTours;
        std::vector<TSP::RoomTour> tspTours;
        rawTours.reserve(rooms.size());
        tspTours.reserve(rooms.size());
        std::size_t failed = 0;
        double rawTotal = 0.0;
        double tspTotal = 0.0;
        for (std::size_t i = 0; i < rooms.size(); ++i) {
            TSP::RoomTour tour = computeRoomTour(rooms[i], radius);
            TSP::RoomTour optimized = TSP::twoOpt(TSP::nearestNeighbor(tour));
            addVisibility(tour, rooms[i].m_footprint);
            addVisibility(optimized, rooms[i].m_footprint);
            std::cout << "[" << (i + 1) << "/" << rooms.size() << "] " << rooms[i].m_name << "  ";
            if (tour.m_ok) {
                const std::size_t blind = std::count_if(tour.m_visPolys.begin(), tour.m_visPolys.end(),
                    [](const std::vector<TSP::Vec2>& ring) { return ring.empty(); });
                std::cout << "steps=" << tour.m_steps << " distance=" << tour.m_distance << " tsp=" << optimized.m_distance;
                if (blind > 0) {
                    std::cout << "  WARN " << blind << " viewpoint(s) without visibility polygon";
                }
                std::cout << "\n";
                rawTotal += tour.m_distance;
                tspTotal += optimized.m_distance;
            } else {
                std::cout << "FAILED (" << tour.m_reason << ")\n";
                ++failed;
            }
            rawTours.push_back(tour);
            tspTours.push_back(optimized);
        }

        writeTours(rawPath, radius, rawTours);
        writeTours(tspPath, radius, tspTours);
        const std::size_t ok = rawTours.size() - failed;
        std::cout << "wrote " << ok << " tours (" << failed << " failed)\n";
        std::cout << "  " << rawPath << "  total=" << rawTotal << "\n";
        std::cout << "  " << tspPath << "  total=" << tspTotal;
        if (rawTotal > 0.0) {
            std::cout << "  (" << (tspTotal - rawTotal) / rawTotal * 100.0 << "%)";
        }
        std::cout << "\n";
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
