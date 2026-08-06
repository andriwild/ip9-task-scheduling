#include "VisLibRational.h"
#include "SightseeingTour1.hpp"

#include <cassert>
#include <iostream>
#include <ranges>
#include <set>
#include <utility>
#include <string>
#include <vector>

#include "tour_io.h"
#include "tsp.h"

// convert a room polygon from double points into VisLib Point
std::vector<VisLib::Point<VisLib::CT>> toVisLibPoints(const std::vector<TSP::Vec2>& polygon) {
    std::vector<VisLib::Point<VisLib::CT>> points;
    points.reserve(polygon.size());
    for (const TSP::Vec2& vertex : polygon) {
        points.emplace_back(VisLib::CT(vertex.m_x), VisLib::CT(vertex.m_y));
    }
    return points;
}

// sightseeingTour may have duplicate points
TSP::RoomTour dropRepeatedStops(TSP::RoomTour tour) {
    std::set<std::pair<double, double>> seen;
    std::vector<TSP::Vec2> stops;
    for (const TSP::Vec2& point : tour.m_path) {
        if (seen.emplace(point.m_x, point.m_y).second) {
            stops.push_back(point);
        }
    }
    tour.m_path  = std::move(stops);
    tour.m_steps = tour.m_path.size();
    return tour;
}

TSP::RoomTour sightseeingTour(const TSP::Room& room, const double radius) {
    const VisLib::SimplePolygon<VisLib::CT> polygon(toVisLibPoints(room.m_footprint));
    VisLib::DCEL<VisLib::CT> dcel(polygon);
    const VisLib::Point<VisLib::CT> start(VisLib::CT(room.m_waypoint.m_x), VisLib::CT(room.m_waypoint.m_y));
    const auto result = dcel.sightseeingTour1(start, radius, VisLib::Show::Nothing);
    assert(!result.m_path.empty());

    TSP::RoomTour tour;
    tour.m_roomName = room.m_name;
    tour.m_start    = TSP::Vec2{room.m_waypoint.m_x, room.m_waypoint.m_y, start};
    tour.m_steps    = result.m_path.size();
    tour.m_distance = static_cast<double>(result.m_distance);
    for (const auto& point : result.m_path) {
        tour.m_path.push_back(TSP::Vec2{static_cast<double>(point.m_x), static_cast<double>(point.m_y), point});
    }
    return tour;
}

void addVisibility(TSP::RoomTour& tour, const std::vector<TSP::Vec2>& footprint) {
    const VisLib::SimplePolygon<VisLib::CT> room(toVisLibPoints(footprint));
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

int main(int argc, char** argv) {
    assert(argc >= 2);
    const double radius = std::stod(argv[1]);

    const std::vector<TSP::Room> rooms = TourIO::readBuilding(TourIO::kBuildingFile);

    std::vector<TSP::RoomTour> tours;

    // for docks, which have no room and therefore no footprint (minimal footprint = 3 edges)
    auto validRooms = rooms | std::views::filter([](const TSP::Room& r) { return r.m_footprint.size() >= 3; });

    for (const TSP::Room& room : validRooms) {
        TSP::RoomTour tour = TSP::twoOpt(TSP::nearestNeighbor(dropRepeatedStops(sightseeingTour(room, radius))));
        addVisibility(tour, room.m_footprint);
        std::cout << tour.m_roomName << "  steps=" << tour.m_steps << "  distance=" << tour.m_distance << "\n";
        tours.push_back(std::move(tour));
    }


    // write the tours into file
    const std::string outPath = TourIO::toursPath(radius);
    TourIO::writeTours(outPath, radius, tours);
    std::cout << "wrote " << tours.size() << " of " << rooms.size() << " rooms -> " << outPath << "\n";
    return 0;
}
