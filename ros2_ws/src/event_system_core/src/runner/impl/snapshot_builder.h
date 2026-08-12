#pragma once

#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "../../sim/ros/path_node.h"
#include "../../util/db.h"
#include "../../util/dist_mat.h"
#include "../../util/log.h"

namespace des {

//TODO: replace with config
const std::string DB_USER     = "wsr_user";
const std::string DB_PASSWORD = "wsr_password";

class SnapshotBuilder {
    DBClient m_db;
    std::shared_ptr<PathPlannerNode> m_planner;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::thread m_rosThread;

public:
    SnapshotBuilder() : m_db({DB_USER, DB_PASSWORD}) {}
    ~SnapshotBuilder() { shutdown(); }

    static int build(const int argc, char* argv[]) {
        rclcpp::init(argc, argv);
        SnapshotBuilder builder;
        const int rc = builder.run();
        rclcpp::shutdown();
        return rc;
    }

    static int buildRooms(const int argc, char* argv[]) {
        rclcpp::init(argc, argv);
        SnapshotBuilder builder;
        const int rc = builder.runRooms();
        rclcpp::shutdown();
        return rc;
    }

    int run() {
        const auto rooms = m_db.rooms();
        if (!rooms.has_value()) {
            throw std::runtime_error("Could not load rooms from DB");
        }
        DES_LOG_INFO("des.snapshot", "Loaded %zu rooms from DB", rooms->size());
        for (const auto& [_, room] : rooms.value()) {
            DES_LOG_DEBUG_STREAM("des.snapshot", room);
        }

        m_planner = std::make_shared<PathPlannerNode>(rooms.value());
        if (!m_planner->isReady()) {
            DES_LOG_ERROR("des.snapshot", "Nav2 planner not available — is planner.sh running?");
            return 1;
        }

        m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        m_rosThread = std::thread([this] {
            m_executor->add_node(m_planner);
            m_executor->spin();
            m_executor->remove_node(m_planner);
        });

        // Matrix index order follows the (name-sorted) RoomMap iteration.
        std::vector<Room> locations;
        locations.reserve(rooms->size());
        for (const auto& [_, loc] : rooms.value()) {
            locations.push_back(loc);
        }

        const bool ok = DistMat::rebuild(locations, m_planner);
        shutdown();
        return ok ? 0 : 1;
    }

    int runRooms() {
        const auto rooms = m_db.rooms();
        if (!rooms.has_value()) {
            throw std::runtime_error("Could not load rooms from DB");
        }
        DES_LOG_INFO("des.snapshot", "Loaded %zu rooms from DB", rooms->size());

        const auto existing = ConfigLoader::loadDistanceMatrix(BUILDING_FILE);
        if (!existing.has_value()) {
            DES_LOG_ERROR("des.snapshot", "No matrix to keep in %s, run the full rebuild", BUILDING_FILE.c_str());
            return 1;
        }
        const auto& [names, mat] = existing.value();

        std::vector<Room> locations;
        locations.reserve(rooms->size());
        for (const auto& [_, loc] : rooms.value()) {
            locations.push_back(loc);
        }

        if (names.size() != locations.size()) {
            DES_LOG_ERROR("des.snapshot", "DB has %zu rooms, the matrix covers %zu, run the full rebuild", locations.size(), names.size());
            return 1;
        }
        for (std::size_t i = 0; i < names.size(); ++i) {
            if (names[i] != locations[i].m_name) {
                DES_LOG_ERROR("des.snapshot", "Room order differs at index %zu ('%s' in the matrix, '%s' in the DB), run the full rebuild", i, names[i].c_str(), locations[i].m_name.c_str());
                return 1;
            }
        }

        const bool ok = DistMat::saveMat(mat, locations);
        DES_LOG_INFO("des.snapshot", "Rooms updated, matrix of %zu x %zu kept: %s", names.size(), names.size(), ok ? "written" : "FAILED");
        return ok ? 0 : 1;
    }

    void shutdown() {
        if (m_executor) {
            m_executor->cancel();
        }
        if (m_rosThread.joinable()) {
            m_rosThread.join();
        }
    }
};

}  // namespace des
