#ifndef NDEBUG
#define NDEBUG
#endif

#include "VisLibRational.h"
#include "SightseeingTour1.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace VisLib;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: tour_gen <radius>   (stdin: line1 'sx sy', line2 'x1 y1 x2 y2 ...')\n";
        return 2;
    }
    const double radius = std::stod(argv[1]);

    std::string startLine;
    std::string polyLine;
    std::getline(std::cin, startLine);
    std::getline(std::cin, polyLine);

    double sx = 0.0;
    double sy = 0.0;
    {
        std::istringstream ss(startLine);
        ss >> sx >> sy;
    }

    std::vector<Point<CT>> pts;
    {
        std::istringstream ss(polyLine);
        double x = 0.0;
        double y = 0.0;
        while (ss >> x >> y) {
            Point<CT> p = Point<CT>(CT(x), CT(y));
            if (!pts.empty() && pts.back() == p) {
                continue;
            }
            pts.push_back(p);
        }
    }
    if (pts.size() >= 2 && pts.front() == pts.back()) {
        pts.pop_back();
    }
    if (pts.size() < 3) {
        std::cout << "DEGENERATE 0 0\n";
        return 0;
    }

    try {
        SimplePolygon<CT> sp(pts);
        DCEL<CT> dcel(sp);
        Point<CT> start = Point<CT>(CT(sx), CT(sy));
        const auto tour = dcel.sightseeingTour1(start, radius, Show::Nothing);
        if (tour.m_path.empty()) {
            std::cout << "DEGENERATE 0 0\n";
            return 0;
        }
        std::cout << "OK " << tour.m_path.size() << " " << static_cast<double>(tour.m_distance) << "\n";
        for (const auto& p : tour.m_path) {
            std::cout << static_cast<double>(p.m_x) << " " << static_cast<double>(p.m_y) << "\n";
        }
    } catch (const std::exception& e) {
        std::cout << "THROW 0 0\n";
        return 0;
    }
    return 0;
}
