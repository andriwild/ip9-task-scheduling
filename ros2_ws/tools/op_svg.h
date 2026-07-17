#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../src/event_system_core/src/algo/op_types.h"

struct Point { float x; float y; };

inline void writeSvg(const std::vector<OpNode>& nodes,
                     const std::vector<Point>& pos,
                     const std::vector<int>& route,
                     const OpParams& params,
                     const std::vector<int>& stations,
                     const std::string& file) {
    float minX = pos[0].x, maxX = pos[0].x, minY = pos[0].y, maxY = pos[0].y;
    for (const auto& p : pos) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    const float margin = 60.0f;
    const float scale  = 14.0f;
    auto sx = [&](float x) { return margin + (x - minX) * scale; };
    auto sy = [&](float y) { return margin + (y - minY) * scale; };
    const float width  = 2 * margin + (maxX - minX) * scale + 140.0f;
    const float height = 2 * margin + (maxY - minY) * scale;

    std::vector<int> tour = { params.startNodeId };
    tour.insert(tour.end(), route.begin(), route.end());
    tour.push_back(params.endNodeId);

    std::ofstream f(file);
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width << "' height='" << height << "'>\n";
    f << "<rect width='100%' height='100%' fill='white'/>\n";

    for (size_t i = 0; i + 1 < tour.size(); ++i) {
        const auto& a = pos[tour[i]];
        const auto& b = pos[tour[i + 1]];
        f << "<line x1='" << sx(a.x) << "' y1='" << sy(a.y)
          << "' x2='" << sx(b.x) << "' y2='" << sy(b.y)
          << "' stroke='#e67e22' stroke-width='2'/>\n";
    }

    auto isStation = [&](int idx) {
        return std::find(stations.begin(), stations.end(), idx) != stations.end();
    };
    for (size_t i = 0; i < nodes.size(); ++i) {
        const int idx = static_cast<int>(i);
        std::string color = "#3498db";
        if (idx == params.startNodeId) { color = "#27ae60"; }
        else if (idx == params.endNodeId) { color = "#c0392b"; }
        else if (isStation(idx)) { color = "#e67e22"; }

        const float radius = std::max(4.0f, nodes[i].reward * 2.0f);
        f << "<circle cx='" << sx(pos[i].x) << "' cy='" << sy(pos[i].y) << "' r='" << radius << "' fill='" << color << "' fill-opacity='0.85'/>\n";
        f << "<text x='" << sx(pos[i].x) + radius + 4 << "' y='" << sy(pos[i].y) + 4
          << "' font-family='sans-serif' font-size='13'>" << nodes[i].name
          << " (r=" << nodes[i].reward << ")</text>\n";
    }

    for (size_t step = 0; step < tour.size(); ++step) {
        const auto& p = pos[tour[step]];
        f << "<text x='" << sx(p.x) - 4 << "' y='" << sy(p.y) - 16
          << "' font-family='sans-serif' font-size='12' fill='#333'>" << step << "</text>\n";
    }

    f << "</svg>\n";
    std::cout << "wrote " << file << "\n";
}
