#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "algo/op_types.h"

struct Point { float x; float y; };

struct SvgStyle {
    std::string room    = "#E4EFF9";
    std::string roomEdge= "#6F9CC4";
    std::string start   = "#E4F0E5";
    std::string startEdge = "#71A177";
    std::string dock    = "#FBEBD7";
    std::string dockEdge= "#D08B3C";
    std::string goal    = "#EDE8F6";
    std::string goalEdge= "#8E79BC";
    std::string tour    = "#3C6E9C";
    std::string line    = "#8A8A8A";
    std::string text    = "#2A2A2A";
    std::string muted   = "#7A7F86";
};

inline void writeSvg(const std::vector<des::OpNode>& nodes,
                     const std::vector<Point>& pos,
                     const std::vector<int>& route,
                     const des::OpParams& params,
                     const std::vector<int>& stations,
                     const std::string& file,
                     const std::vector<char>& seiten = {}) {
    const SvgStyle st;
    float minX = pos[0].x, maxX = pos[0].x, minY = pos[0].y, maxY = pos[0].y;
    for (const auto& p : pos) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    const float scale  = 11.0f;
    const float pad    = 150.0f;   // linker und oberer Rand
    const float padR   = 270.0f;   // rechts, fuer die Beschriftungen
    const float legendH = 56.0f;
    auto sx = [&](float x) { return pad + (x - minX) * scale; };
    auto sy = [&](float y) { return pad + (maxY - y) * scale; };
    const float width  = pad + padR + (maxX - minX) * scale;
    const float height = pad + 60.0f + (maxY - minY) * scale + legendH;   // unten knapper als oben

    const float cx = (minX + maxX) / 2.0f;

    std::vector<int> tour = { params.startNodeId };
    tour.insert(tour.end(), route.begin(), route.end());
    if (!params.openEnd) { tour.push_back(params.endNodeId); }

    auto isStation = [&](int idx) {
        return std::find(stations.begin(), stations.end(), idx) != stations.end();
    };
    auto radius = [&](std::size_t i) {
        const int idx = static_cast<int>(i);
        if (idx == params.startNodeId || idx == params.endNodeId || isStation(idx)) {
            return 11.0f;
        }
        // Kreisflaeche proportional zum Nutzen, Skala bis zum Gewicht 1.0
        return 8.0f + 24.0f * std::sqrt(nodes[i].reward);
    };

    std::ofstream f(file);
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width << "' height='" << height
      << "' viewBox='0 0 " << width << " " << height << "' font-family='Helvetica, Arial, sans-serif'>\n";
    f << "<rect width='100%' height='100%' fill='#FFFFFF'/>\n";

    for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
        const auto& a = pos[tour[i]];
        const auto& b = pos[tour[i + 1]];
        f << "<line x1='" << sx(a.x) << "' y1='" << sy(a.y) << "' x2='" << sx(b.x) << "' y2='" << sy(b.y)
          << "' stroke='" << st.line << "' stroke-width='2.6' stroke-linecap='round'/>\n";
    }

    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const int idx = static_cast<int>(i);
        std::string fill = st.room, edge = st.roomEdge;
        if (idx == params.startNodeId)      { fill = st.start; edge = st.startEdge; }
        else if (idx == params.endNodeId)   { fill = st.goal;  edge = st.goalEdge; }
        else if (isStation(idx))            { fill = st.dock;  edge = st.dockEdge; }

        const float r = radius(i);
        if (isStation(idx)) {
            f << "<rect x='" << sx(pos[i].x) - r << "' y='" << sy(pos[i].y) - r << "' width='" << 2 * r
              << "' height='" << 2 * r << "' rx='4' fill='" << fill << "' stroke='" << edge << "' stroke-width='1.8'/>\n";
        } else {
            f << "<circle cx='" << sx(pos[i].x) << "' cy='" << sy(pos[i].y) << "' r='" << r
              << "' fill='" << fill << "' stroke='" << edge << "' stroke-width='1.8'/>\n";
        }

        const char seite = i < seiten.size() ? seiten[i] : (pos[i].x > cx ? 'l' : 'r');
        float tx = sx(pos[i].x) + r + 9, ty = sy(pos[i].y) + 5;
        std::string anchor = "start";
        if (seite == 'l') { tx = sx(pos[i].x) - r - 9; anchor = "end"; }
        else if (seite == 'd') { tx = sx(pos[i].x); ty = sy(pos[i].y) + r + 22; anchor = "middle"; }
        f << "<text x='" << tx << "' y='" << ty << "' font-size='21' fill='" << st.text
          << "' text-anchor='" << anchor << "'>" << nodes[i].name;
        if (nodes[i].reward > 0.0f) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.2f", nodes[i].reward);
            f << "<tspan fill='" << st.muted << "'>&#160;&#160;<tspan font-style='italic'>r</tspan>"
              << "<tspan font-size='16' dy='4'>i</tspan><tspan dy='-4'>&#160;=&#160;" << buf
              << "</tspan></tspan>";
        }
        f << "</text>\n";
    }

    for (std::size_t step = 0; step < tour.size(); ++step) {
        const auto& p = pos[tour[step]];
        const float r = radius(tour[step]);
        f << "<circle cx='" << sx(p.x) << "' cy='" << sy(p.y) - r - 11 << "' r='12' fill='" << st.tour << "'/>\n";
        f << "<text x='" << sx(p.x) << "' y='" << sy(p.y) - r - 6 << "' font-size='18' fill='#FFFFFF'"
          << " text-anchor='middle'>" << step << "</text>\n";
    }

    const float ly = height - legendH + 22.0f;
    float lx = pad;
    struct Item { std::string fill, edge, label; };
    const std::vector<Item> items = {
        { st.start, st.startEdge, "Start" },
        { st.room,  st.roomEdge,  "Raum, Fläche nach Nutzen" },
        { st.dock,  st.dockEdge,  "Dock" },
        { st.goal,  st.goalEdge,  "Ende am nächsten Termin" },
    };
    for (const auto& it : items) {
        if (it.label == "Dock") {
            f << "<rect x='" << lx << "' y='" << ly - 12 << "' width='16' height='16' rx='3' fill='" << it.fill
              << "' stroke='" << it.edge << "' stroke-width='1.8'/>\n";
        } else {
            f << "<circle cx='" << lx + 8 << "' cy='" << ly - 4 << "' r='8' fill='" << it.fill
              << "' stroke='" << it.edge << "' stroke-width='1.8'/>\n";
        }
        f << "<text x='" << lx + 22 << "' y='" << ly << "' font-size='21' fill='" << st.text << "'>"
          << it.label << "</text>\n";
        lx += 22.0f + it.label.size() * 10.0f + 26.0f;
    }

    f << "</svg>\n";
    std::cout << "wrote " << file << "\n";
}
