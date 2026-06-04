#pragma once

#include <filesystem>
#include <fstream>
#include <iomanip>

#include "db.h"
#include "init/config_loader.h"
#include "sim/ros/path_node.h"

const std::string DIST_MAT_FILE = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/dist_mat.json";
constexpr float DIST_SENTINEL = -1.0f;

// Symmetric N x N path-distance cache between all waypoints.
//
// mat[i][i] == 0                   — diagonal
// mat[i][j] >  0                   — cached path distance (metres, Nav2)
// mat[i][j] == DIST_SENTINEL (-1)  — not yet computed OR unreachable
// mat[i][j] == mat[j][i]           — symmetric (planner is called only for i<j)
//
// Persistence: serialised to DIST_MAT_FILE as
//   {
//     "mat":       [[...], [...], ...],          // N x N distances
//     "names":     ["wp_a", "wp_b", ...],        // index → waypoint name
//     "locations": [{"x":.., "y":.., "yaw":..}, ...]  // index → world pose
//   }
//
// The names array (and only the names) invalidates the cache: a mismatch in
// size/order triggers a full recompute. Sentinel cells let an interrupted run
// resume without redoing finished entries.
//
// "locations" is metadata for downstream consumers (algorithm development,
// visualisation tools, sanity checks) — it does NOT participate in cache
// validity. Older cached files without "locations" stay valid; the next
// successful save upgrades them in place.

using Mat = std::vector<std::vector<float>>;

class DistMat {
    DBClient& m_db;

    static bool namesMatch(const nlohmann::json& j, const std::vector<des::Location>& points) {
        if (!j.contains("names")) {
            return false;
        }
        const auto& names = j.at("names");
        if (!names.is_array() || names.size() != points.size()) {
            return false;
        }

        for (size_t i = 0; i < points.size(); ++i) {
            if (names.at(i).get<std::string>() != points.at(i).m_name) return false;
        }
        return true;
    }

public:
    explicit DistMat(DBClient& db): m_db(db) {}

    static bool saveMat(const Mat& mat, const std::vector<des::Location>& points) {
        nlohmann::json j;
        j["mat"] = mat;

        std::vector<std::string> names;
        names.reserve(points.size());
        nlohmann::json locations = nlohmann::json::array();
        for (const auto& p : points) {
            names.push_back(p.m_name);
            locations.push_back({
                {"x",   p.m_p.m_x},
                {"y",   p.m_p.m_y},
                {"yaw", p.m_p.m_yaw},
            });
        }
        j["names"]     = names;
        j["locations"] = locations;

        std::ofstream file(DIST_MAT_FILE);
        if (!file.is_open()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.dist_mat"), "Could not write distance matrix to: %s", DIST_MAT_FILE.c_str());
            return false;
        }
        file << std::setw(4) << j << std::endl;
        return true;
    }

    Mat getMat(std::shared_ptr<PathPlannerNode> planner) {
        DES_LOG_INFO(rclcpp::get_logger("des.dist_mat"), "--- DISTANCE MATRIX ---");

        auto pointsOpt = m_db.waypoints();
        assert(pointsOpt.has_value());

        const auto points = pointsOpt.value();
        const size_t nPoints = points.size();

        // Load cached matrix if file exists and waypoint set (names+order) still matches.
        Mat mat;
        bool needsUpgrade = false;
        if (std::filesystem::exists(DIST_MAT_FILE)) {
            auto json = ConfigLoader::getJson(DIST_MAT_FILE);
            if (json.has_value() && json.value().contains("mat") && namesMatch(json.value(), points)) {
                mat = json.value().at("mat").get<Mat>();
                DES_LOG_INFO(rclcpp::get_logger("des.dist_mat"), "Loaded cached distance matrix (%zu x %zu)", mat.size(), mat.empty() ? 0 : mat.front().size());
                // Older cache files predate the "locations" field — upgrade in place
                // without recomputing any distances.
                needsUpgrade = !json.value().contains("locations");
            } else {
                DES_LOG_WARN(rclcpp::get_logger("des.dist_mat"), "Cached distance matrix does not match current waypoints — recomputing");
            }
        }

        // Ensure mat is nPoints x nPoints, filled with sentinel where unknown, 0 on diagonal.
        mat.resize(nPoints, std::vector<float>(nPoints, DIST_SENTINEL));
        for (auto& row : mat) {
            row.resize(nPoints, DIST_SENTINEL);
        }
        for (size_t i = 0; i < nPoints; ++i) {
            mat[i][i] = 0.0f;
        }

        // One-time rewrite to attach the new locations metadata. Distances are
        // unchanged, so no waypoint pair is recomputed.
        if (needsUpgrade) {
            DES_LOG_INFO(rclcpp::get_logger("des.dist_mat"), "Upgrading cache file with waypoint locations (no recompute)");
            saveMat(mat, points);
        }

        // Fill upper triangle, mirror to lower
        for (size_t i = 0; i < nPoints; ++i) {
            bool rowDirty = false;
            for (size_t j = i + 1; j < nPoints; ++j) {

                // Only calc if a sentinel is present
                if (mat[i][j] < 0.0f) {
                    const auto& p1 = points.at(i);
                    const auto& p2 = points.at(j);
                    DES_LOG_INFO(rclcpp::get_logger("des.dist_mat"), "mat calc (%zu, %zu) %s | %s", i, j, p1.m_name.c_str(), p2.m_name.c_str());

                    auto d = planner->calcDistance(p1.m_name, p2.m_name, false);
                    assert(d.has_value());

                    mat[i][j] = static_cast<float>(d.value());
                    mat[j][i] = mat[i][j];
                    rowDirty = true;
                }
            }
            if (rowDirty) {
                saveMat(mat, points);
            }
        }

        DES_LOG_INFO(rclcpp::get_logger("des.dist_mat"), "Distance matrix ready (%zu x %zu)", nPoints, nPoints);
        return mat;
    }
};
