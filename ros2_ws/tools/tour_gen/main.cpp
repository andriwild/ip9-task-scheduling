#include "VisLibRational.h"
#include "SightseeingTour1.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "tour_io.h"
#include "tsp.h"

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
    
    // for docks, which have no room and therefore no footprint
    if (room.m_footprint.size() < 3) {
        tour.m_reason = "no-footprint";
        return tour;
    }

    TSP::RoomTour result = computeTour(room.m_footprint, room.m_waypoint, radius);
    result.m_roomName = room.m_name;
    return result;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: tour_gen <radius>   (run from the workspace root)\n";
        return 2;
    }
    const double radius = std::stod(argv[1]);
    const std::string outPath = TourIO::toursFile(radius);

    try {
        const std::vector<TSP::Room> rooms = TourIO::readBuilding(TourIO::kBuildingFile);
        std::cout << "read " << rooms.size() << " rooms from " << TourIO::kBuildingFile << "\n";

        std::vector<TSP::RoomTour> tours;
        tours.reserve(rooms.size());
        std::size_t failed = 0;
        double total = 0.0;
        for (std::size_t i = 0; i < rooms.size(); ++i) {
            TSP::RoomTour tour = TSP::twoOpt(TSP::nearestNeighbor(computeRoomTour(rooms[i], radius)));
            addVisibility(tour, rooms[i].m_footprint);
            std::cout << "[" << (i + 1) << "/" << rooms.size() << "] " << rooms[i].m_name << "  ";
            if (tour.m_ok) {
                const std::size_t blind = std::count_if(tour.m_visPolys.begin(), tour.m_visPolys.end(),
                    [](const std::vector<TSP::Vec2>& ring) { return ring.empty(); });
                std::cout << "steps=" << tour.m_steps << " distance=" << tour.m_distance;
                if (blind > 0) {
                    std::cout << "  WARN " << blind << " viewpoint(s) without visibility polygon";
                }
                std::cout << "\n";
                total += tour.m_distance;
            } else {
                std::cout << "FAILED (" << tour.m_reason << ")\n";
                ++failed;
            }
            tours.push_back(tour);
        }

        TourIO::writeTours(outPath, radius, tours);
        const std::size_t ok = tours.size() - failed;
        std::cout << "wrote " << ok << " tours (" << failed << " failed)\n";
        std::cout << "  " << outPath << "  total=" << total << "\n";
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
