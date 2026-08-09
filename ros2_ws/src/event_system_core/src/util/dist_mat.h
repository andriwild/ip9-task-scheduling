#pragma once

#include <ctime>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <memory>

#include "init/config_loader.h"
#include "sim/ros/path_node.h"

// Symmetric N x N path-distance matrix between all waypoints, persisted as part
// of the building snapshot (BUILDING_FILE):
//
//   {
//     "names":     ["wp_a", "wp_b", ...],              // index -> waypoint name
//     "mat":       [[...], [...], ...],                // N x N path distances (m, Nav2)
//     "locations": [{"x":.., "y":.., "yaw":..}, ...],  // index -> world pose
//     "areas":     [12.5, null, ...]                   // index -> zone area (m²) or null
//   }
//
// mat[i][i] == 0, mat[i][j] == mat[j][i]. The file is a fully generated artifact:
// rebuild() recomputes every pair from scratch (the building changes rarely, so
// there is no incremental update). The running sim never writes it.

namespace des {


class DistMat {
public:
    // Writes the full snapshot. Areas come straight from each Location's m_area
    // (null when a waypoint has no matching search zone).
    static bool saveMat(const Mat& mat, const std::vector<Room>& points) {
        nlohmann::json rooms = nlohmann::json::array();
        for (const Room& p : points) {
            nlohmann::json entry{
                { "name", p.m_name },
                { "x",    p.m_waypoint.m_x },
                { "y",    p.m_waypoint.m_y },
                { "yaw",  p.m_waypoint.m_yaw },
                { "type", roomTypeToString(p.m_roomType) },
            };
            if (p.m_area) {
                entry["area"] = *p.m_area;
            }
            for (const Point& v : p.m_footprint) {
                entry["footprint"].push_back({ v.m_x, v.m_y });
            }
            rooms.push_back(std::move(entry));
        }

        const std::time_t now = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

        nlohmann::json j;
        j["generated_at"] = stamp;
        j["rooms"]        = std::move(rooms);
        j["mat"]          = mat;

        std::ofstream file(BUILDING_FILE);
        assert(file.is_open());
        file << std::setw(4) << j << std::endl;
        return true;
    }

    // Recomputes the entire matrix from scratch via the Nav2 planner and writes the snapshot.
    // `locations` must already carry coordinates + areas (the DB view from DBClient::rooms).
    static bool rebuild(const std::vector<Room>& locations, std::shared_ptr<PathPlannerNode> planner) {
        const size_t n = locations.size();
        DES_LOG_INFO("des.dist_mat", "--- REBUILD DISTANCE MATRIX (%zu x %zu) ---", n, n);

        Mat mat(n, std::vector<float>(n, 0.0f));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const auto& p1 = locations.at(i);
                const auto& p2 = locations.at(j);
                DES_LOG_INFO("des.dist_mat", "mat calc (%zu, %zu) %s | %s", i, j, p1.m_name.c_str(), p2.m_name.c_str());

                const auto d = planner->calcDistance(p1.m_name, p2.m_name, false);
                if (!d.has_value()) {
                    DES_LOG_ERROR("des.dist_mat", "No path for %s -> %s; aborting rebuild", p1.m_name.c_str(), p2.m_name.c_str());
                    return false;
                }
                mat[i][j] = static_cast<float>(d.value());
                mat[j][i] = mat[i][j];
            }
        }

        const bool ok = saveMat(mat, locations);
        DES_LOG_DEBUG("des.dist_mat", "Building snapshot %s", ok ? "written" : "FAILED");
        return ok;
    }
};

}  // namespace des
