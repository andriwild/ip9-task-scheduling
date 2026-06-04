#pragma once

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

using Mat = std::vector<std::vector<float>>;

class DistMat {
public:
    // Writes the full snapshot. Areas come straight from each Location's m_area
    // (null when a waypoint has no matching search zone).
    static bool saveMat(const Mat& mat, const std::vector<des::Location>& points) {
        nlohmann::json j;
        j["mat"] = mat;

        std::vector<std::string> names;
        names.reserve(points.size());
        nlohmann::json locations = nlohmann::json::array();
        nlohmann::json areas     = nlohmann::json::array();
        for (const auto& p : points) {
            names.push_back(p.m_name);
            locations.push_back({
                {"x",   p.m_p.m_x},
                {"y",   p.m_p.m_y},
                {"yaw", p.m_p.m_yaw},
            });
            if (p.m_area) areas.push_back(*p.m_area);
            else          areas.push_back(nullptr);
        }
        j["names"]     = names;
        j["locations"] = locations;
        j["areas"]     = areas;

        std::ofstream file(BUILDING_FILE);
        if (!file.is_open()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.dist_mat"), "Could not write building snapshot to: %s", BUILDING_FILE.c_str());
            return false;
        }
        file << std::setw(4) << j << std::endl;
        return true;
    }

    // Recomputes the entire matrix from scratch via the Nav2 planner and writes
    // the snapshot. `locations` must already carry coordinates + areas (the DB
    // view from loadLocationsFromDB). Requires the planner server to be running.
    static bool rebuild(const std::vector<des::Location>& locations, std::shared_ptr<PathPlannerNode> planner) {
        const size_t n = locations.size();
        DES_LOG_INFO(rclcpp::get_logger("des.dist_mat"), "--- REBUILD DISTANCE MATRIX (%zu x %zu) ---", n, n);

        Mat mat(n, std::vector<float>(n, 0.0f));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const auto& p1 = locations.at(i);
                const auto& p2 = locations.at(j);
                DES_LOG_INFO(rclcpp::get_logger("des.dist_mat"), "mat calc (%zu, %zu) %s | %s", i, j, p1.m_name.c_str(), p2.m_name.c_str());

                const auto d = planner->calcDistance(p1.m_name, p2.m_name, false);
                if (!d.has_value()) {
                    DES_LOG_ERROR(rclcpp::get_logger("des.dist_mat"), "No path for %s -> %s; aborting rebuild", p1.m_name.c_str(), p2.m_name.c_str());
                    return false;
                }
                mat[i][j] = static_cast<float>(d.value());
                mat[j][i] = mat[i][j];
            }
        }

        const bool ok = saveMat(mat, locations);
        DES_LOG_INFO(rclcpp::get_logger("des.dist_mat"), "Building snapshot %s", ok ? "written" : "FAILED");
        return ok;
    }
};
