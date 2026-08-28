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
    std::string room      = "#BCD8F0";   // besuchter Raum
    std::string roomEdge  = "#3C6E9C";
    std::string skip      = "#F0F1F3";   // uebergangener Raum
    std::string skipEdge  = "#B6BBC2";
    std::string start     = "#E4F0E5";
    std::string startEdge = "#71A177";
    std::string dock      = "#FBEBD7";
    std::string dockEdge  = "#D08B3C";
    std::string goal      = "#EDE8F6";
    std::string goalEdge  = "#8E79BC";
    std::string tour      = "#3C6E9C";
    std::string line      = "#3C6E9C";
    std::string text      = "#2A2A2A";
    std::string muted     = "#9AA0A8";
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
    const float scale   = 11.0f;
    const float pad     = 120.0f;   // linker Rand
    const float padTop  = 70.0f;    // oberer Rand
    const float padR    = 120.0f;   // rechts, fuer die Beschriftungen
    const float legendH = 56.0f;
    auto sx = [&](float x) { return pad + (x - minX) * scale; };
    auto sy = [&](float y) { return padTop + (maxY - y) * scale; };
    const float width  = pad + padR + (maxX - minX) * scale;
    const float height = padTop + 70.0f + (maxY - minY) * scale + legendH;

    const float cx = (minX + maxX) / 2.0f;

    std::vector<int> tour = { params.startNodeId };
    tour.insert(tour.end(), route.begin(), route.end());
    if (!params.openEnd) { tour.push_back(params.endNodeId); }

    auto isStation = [&](int idx) {
        return std::find(stations.begin(), stations.end(), idx) != stations.end();
    };
    auto besucht = [&](int idx) {
        return std::find(tour.begin(), tour.end(), idx) != tour.end();
    };

    float maxReward = 0.0f;
    for (const auto& nd : nodes) { maxReward = std::max(maxReward, nd.reward); }
    if (maxReward <= 0.0f) { maxReward = 1.0f; }

    // Kreisflaeche proportional zum Nutzen, der groesste Kandidat gibt die Skala vor
    auto radius = [&](std::size_t i) {
        const int idx = static_cast<int>(i);
        if (idx == params.startNodeId || idx == params.endNodeId || isStation(idx)) {
            return 11.0f;
        }
        return 10.0f + 26.0f * std::sqrt(nodes[i].reward / maxReward);
    };

    std::ofstream f(file);
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width << "' height='" << height
      << "' viewBox='0 0 " << width << " " << height << "' font-family='Helvetica, Arial, sans-serif'>\n";
    f << "<rect width='100%' height='100%' fill='#FFFFFF'/>\n";

    // Die Tour verbindet Raumschwerpunkte, der Roboter faehrt in Wirklichkeit
    // durch die Gaenge. Eine gerade Verbindung laeuft deshalb gelegentlich
    // mitten durch einen uebergangenen Raum und sieht aus wie ein Besuch.
    // Solche Kanten werden zu einem flachen Bogen um den Knoten herum.
    auto ausweichen = [&](int ia, int ib) {
        const float ax = sx(pos[ia].x), ay = sy(pos[ia].y);
        const float bx = sx(pos[ib].x), by = sy(pos[ib].y);
        const float dx = bx - ax, dy = by - ay;
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.0f) { return 0.0f; }
        const float ux = dx / len, uy = dy / len;
        const float nx = -uy, ny = ux;

        // Der Bogen weicht in der Mitte um den gesuchten Versatz aus, an den
        // Enden weniger. Der Faktor g rechnet die noetige Luecke beim Knoten
        // auf den Versatz in der Mitte um.
        const float grenze = 90.0f;
        std::vector<std::pair<float, float>> verboten;
        for (std::size_t k = 0; k < nodes.size(); ++k) {
            const int idx = static_cast<int>(k);
            if (idx == ia || idx == ib) { continue; }
            const float vx = sx(pos[k].x) - ax, vy = sy(pos[k].y) - ay;
            const float t  = (vx * ux + vy * uy) / len;
            if (t < 0.15f || t > 0.85f) { continue; }
            const float g    = 4.0f * t * (1.0f - t);
            const float quer = vx * nx + vy * ny;
            const float frei = radius(k) + 16.0f;
            const float lo   = (quer - frei) / g;
            const float hi   = (quer + frei) / g;
            if (lo > grenze || hi < -grenze) { continue; }
            verboten.emplace_back(lo, hi);
        }
        if (verboten.empty()) { return 0.0f; }

        std::vector<float> kandidaten = { 0.0f };
        for (const auto& iv : verboten) {
            kandidaten.push_back(iv.first);
            kandidaten.push_back(iv.second);
        }
        float bester = 0.0f;
        bool gefunden = false;
        for (const float c : kandidaten) {
            if (std::fabs(c) > grenze) { continue; }
            bool frei = true;
            for (const auto& iv : verboten) {
                if (c > iv.first + 0.5f && c < iv.second - 0.5f) { frei = false; break; }
            }
            if (!frei) { continue; }
            if (!gefunden || std::fabs(c) < std::fabs(bester)) { bester = c; gefunden = true; }
        }
        return gefunden ? bester : 0.0f;
    };

    for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
        const int ia = tour[i], ib = tour[i + 1];
        const float ax = sx(pos[ia].x), ay = sy(pos[ia].y);
        const float bx = sx(pos[ib].x), by = sy(pos[ib].y);
        const float versatz = ausweichen(ia, ib);
        if (std::fabs(versatz) < 0.5f) {
            f << "<line x1='" << ax << "' y1='" << ay << "' x2='" << bx << "' y2='" << by
              << "' stroke='" << st.line << "' stroke-width='3.0' stroke-linecap='round'"
              << " opacity='0.85'/>\n";
            continue;
        }
        const float dx = bx - ax, dy = by - ay;
        const float len = std::sqrt(dx * dx + dy * dy);
        const float nx = -dy / len, ny = dx / len;
        const float ccx = (ax + bx) / 2.0f + 2.0f * versatz * nx;
        const float ccy = (ay + by) / 2.0f + 2.0f * versatz * ny;
        f << "<path d='M " << ax << " " << ay << " Q " << ccx << " " << ccy << " " << bx << " " << by
          << "' fill='none' stroke='" << st.line << "' stroke-width='3.0' stroke-linecap='round'"
          << " opacity='0.85'/>\n";
    }

    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const int idx = static_cast<int>(i);
        const bool drin = besucht(idx);
        std::string fill = drin ? st.room : st.skip;
        std::string edge = drin ? st.roomEdge : st.skipEdge;
        if (idx == params.startNodeId)      { fill = st.start; edge = st.startEdge; }
        else if (idx == params.endNodeId)   { fill = st.goal;  edge = st.goalEdge; }
        else if (isStation(idx))            { fill = st.dock;  edge = st.dockEdge; }

        const float r = radius(i);
        if (isStation(idx)) {
            f << "<rect x='" << sx(pos[i].x) - r << "' y='" << sy(pos[i].y) - r << "' width='" << 2 * r
              << "' height='" << 2 * r << "' rx='4' fill='" << fill << "' stroke='" << edge
              << "' stroke-width='1.8'/>\n";
        } else {
            f << "<circle cx='" << sx(pos[i].x) << "' cy='" << sy(pos[i].y) << "' r='" << r
              << "' fill='" << fill << "' stroke='" << edge << "' stroke-width='1.8'/>\n";
        }

        // Nummer des Raums in den Kreis, dort stoert sie keine Nachbarn
        const bool kandidat = nodes[i].reward > 0.0f;
        if (kandidat) {
            f << "<text x='" << sx(pos[i].x) << "' y='" << sy(pos[i].y) + 7 << "' font-size='20'"
              << " fill='" << (drin ? st.text : st.muted) << "' text-anchor='middle'>"
              << nodes[i].name << "</text>\n";
        }

        // Beschriftung daneben, beim Kandidaten der Nutzen, sonst der Name
        const char seite = i < seiten.size() ? seiten[i] : (pos[i].x > cx ? 'l' : 'r');
        float tx = sx(pos[i].x) + r + 9, ty = sy(pos[i].y) + 6;
        std::string anchor = "start";
        if (seite == 'l')      { tx = sx(pos[i].x) - r - 9; anchor = "end"; }
        else if (seite == 'd') { tx = sx(pos[i].x); ty = sy(pos[i].y) + r + 24; anchor = "middle"; }
        else if (seite == 'o') { tx = sx(pos[i].x); ty = sy(pos[i].y) - r - (drin ? 36 : 14); anchor = "middle"; }

        f << "<text x='" << tx << "' y='" << ty << "' font-size='21' text-anchor='" << anchor << "'";
        if (kandidat) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.1f", nodes[i].reward);
            f << " fill='" << (drin ? st.text : st.muted) << "'>"
              << "<tspan font-style='italic'>r</tspan> = " << buf << "</text>\n";
        } else {
            f << " fill='" << st.text << "'>" << nodes[i].name << "</text>\n";
        }
    }

    // Reihenfolge der Tour als Marke ueber dem Knoten
    for (std::size_t step = 0; step < tour.size(); ++step) {
        const auto& p = pos[tour[step]];
        const float r = radius(tour[step]);
        f << "<circle cx='" << sx(p.x) << "' cy='" << sy(p.y) - r - 12 << "' r='13' fill='" << st.tour
          << "'/>\n";
        f << "<text x='" << sx(p.x) << "' y='" << sy(p.y) - r - 6 << "' font-size='18' fill='#FFFFFF'"
          << " text-anchor='middle'>" << step << "</text>\n";
    }

    const float ly = height - legendH + 22.0f;
    float lx = pad;
    struct Item { std::string fill, edge, label; };
    const std::vector<Item> items = {
        { st.start, st.startEdge, "Start" },
        { st.room,  st.roomEdge,  "besucht, Fl\xC3\xA4""che nach Nutzen" },
        { st.skip,  st.skipEdge,  "\xC3\xBC""bergangen" },
        { st.dock,  st.dockEdge,  "Dock" },
        { st.goal,  st.goalEdge,  "Termin" },
    };
    for (const auto& it : items) {
        if (it.label == "Dock") {
            f << "<rect x='" << lx << "' y='" << ly - 12 << "' width='16' height='16' rx='3' fill='"
              << it.fill << "' stroke='" << it.edge << "' stroke-width='1.8'/>\n";
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
