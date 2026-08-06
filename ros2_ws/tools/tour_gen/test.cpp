#include "tsp.h"

#include <iomanip>
#include <iostream>
#include <random>

int failures = 0;

void printTSP(const std::string& label, const TSP::RoomTour& tour) {
    std::cout << label << "  steps=" << tour.m_steps << "  distance=" << tour.m_distance << "\n";
    for (const TSP::Vec2& point : tour.m_path) {
        std::cout << "    " << point.m_x << " " << point.m_y << "\n";
    }
}

void check(const std::string& label, const bool ok) {
    std::cout << (ok ? "  PASS  " : "  FAIL  ") << label << "\n";
    if (!ok) {
        ++failures;
    }
}

double walkLength(const TSP::RoomTour& tour) {
    double d = 0.0;
    for (std::size_t i = 1; i < tour.m_path.size(); ++i) {
        d += TSP::distance(tour.m_path[i - 1], tour.m_path[i]);
    }
    return d;
}

bool isClosed(const TSP::RoomTour& tour) {
    return !tour.m_path.empty()
        && tour.m_path.front().m_x == tour.m_path.back().m_x
        && tour.m_path.front().m_y == tour.m_path.back().m_y;
}

TSP::RoomTour makeTour(const std::vector<TSP::Vec2>& path) {
    TSP::RoomTour tour;
    tour.m_roomName = "test";
    tour.m_start = path.front();
    tour.m_path = path;
    tour.m_steps = path.size();
    tour.m_distance = walkLength(tour);
    return tour;
}

void testNearestNeighbor() {
    std::cout << "nearestNeighbor: scrambled square\n";
    const TSP::RoomTour tour = makeTour({{0, 0}, {10, 10}, {10, 0}, {0, 10}, {0, 0}});
    const TSP::RoomTour result = TSP::nearestNeighbor(tour);

    printTSP("  before", tour);
    printTSP("  after ", result);

    check("closed", isClosed(result));
    check("shorter", result.m_distance < tour.m_distance);
    check("point count kept", result.m_path.size() == tour.m_path.size());
    check("steps match path", result.m_steps == result.m_path.size());
    check("distance matches path", std::abs(result.m_distance - walkLength(result)) < 1e-9);
    check("perimeter found", std::abs(result.m_distance - 40.0) < 1e-9);
}

void testTwoOptFixesCrossing() {
    std::cout << "twoOpt: crossing square\n";
    const TSP::RoomTour tour = makeTour({{0, 0}, {10, 10}, {10, 0}, {0, 10}, {0, 0}});
    const TSP::RoomTour result = TSP::twoOpt(tour);

    printTSP("  before", tour);
    printTSP("  after ", result);

    check("closed", isClosed(result));
    check("shorter", result.m_distance < tour.m_distance);
    check("point count kept", result.m_path.size() == tour.m_path.size());
    check("distance matches path", std::abs(result.m_distance - walkLength(result)) < 1e-9);
    check("perimeter found", std::abs(result.m_distance - 40.0) < 1e-9);
}

void testTwoOptKeepsTourClosed() {
    std::cout << "twoOpt: closing point must survive every reversal\n";
    const std::vector<std::vector<TSP::Vec2>> paths = {
        {{0, 0}, {3, 4}, {12, 5}, {4, 3}, {1, 5}, {0, 6}, {0, 0}},
        {{0, 0}, {4, 9}, {9, 1}, {1, 8}, {8, 8}, {2, 2}, {0, 0}},
        {{0, 0}, {2, 9}, {11, 5}, {5, 4}, {2, 1}, {6, 7}, {11, 4}, {3, 5}, {0, 0}},
        {{0, 0}, {11, 2}, {2, 1}, {7, 4}, {3, 11}, {5, 5}, {12, 10}, {4, 4}, {6, 7}, {0, 0}},
    };
    for (std::size_t i = 0; i < paths.size(); ++i) {
        const TSP::RoomTour tour = makeTour(paths[i]);
        const TSP::RoomTour nn = TSP::nearestNeighbor(tour);
        const TSP::RoomTour opt = TSP::twoOpt(nn);
        const std::string tag = "path " + std::to_string(i) + ": ";

        bool reordered = false;
        for (std::size_t k = 0; k < nn.m_path.size(); ++k) {
            if (nn.m_path[k].m_x != opt.m_path[k].m_x || nn.m_path[k].m_y != opt.m_path[k].m_y) {
                reordered = true;
            }
        }

        check(tag + "twoOpt actually reordered", reordered);
        check(tag + "closed after nearestNeighbor", isClosed(nn));
        check(tag + "closed after twoOpt", isClosed(opt));
        check(tag + "start unchanged", opt.m_path.front().m_x == tour.m_start.m_x && opt.m_path.front().m_y == tour.m_start.m_y);
        check(tag + "point count kept", opt.m_path.size() == tour.m_path.size());
        check(tag + "shorter than nearestNeighbor", opt.m_distance < nn.m_distance);
        check(tag + "distance matches path", std::abs(opt.m_distance - walkLength(opt)) < 1e-6);
    }
}

void testDistanceStaysExactOnLargeTour() {
    std::cout << "twoOpt: accumulated distance must not drift on a corridor-sized tour\n";
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> uni(0.0, 90.0);
    std::vector<TSP::Vec2> path;
    path.push_back(TSP::Vec2{0.0, 0.0});
    for (int i = 0; i < 107; ++i) {
        path.push_back(TSP::Vec2{uni(gen), uni(gen)});
    }
    path.push_back(TSP::Vec2{0.0, 0.0});

    const TSP::RoomTour tour = makeTour(path);
    const TSP::RoomTour opt = TSP::twoOpt(TSP::nearestNeighbor(tour));
    const double drift = std::abs(opt.m_distance - walkLength(opt));

    std::cout << "  points=" << opt.m_path.size() << "  distance=" << opt.m_distance << "  drift=" << std::scientific << drift << std::fixed << "\n";
    check("closed", isClosed(opt));
    check("point count kept", opt.m_path.size() == tour.m_path.size());
    check("distance matches path within 1e-9", drift < 1e-9);
}

void testGuards() {
    std::cout << "guards: short and empty tours pass through untouched\n";
    TSP::RoomTour single = makeTour({{3, 4}});
    check("single point survives nearestNeighbor", TSP::nearestNeighbor(single).m_path.size() == 1);
    check("single point survives twoOpt", TSP::twoOpt(single).m_path.size() == 1);

    TSP::RoomTour empty;
    check("empty tour survives nearestNeighbor", TSP::nearestNeighbor(empty).m_path.empty());
    check("empty tour survives twoOpt", TSP::twoOpt(empty).m_path.empty());
}

int main() {
    std::cout << std::fixed << std::setprecision(4);
    testNearestNeighbor();
    testTwoOptFixesCrossing();
    testTwoOptKeepsTourClosed();
    testDistanceStaysExactOnLargeTour();
    testGuards();

    std::cout << (failures == 0 ? "ALL CHECKS PASSED\n" : std::to_string(failures) + " CHECKS FAILED\n");
    return failures == 0 ? 0 : 1;
}
