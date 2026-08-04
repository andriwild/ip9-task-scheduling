#pragma once

#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "../../sim/ros/path_node.h"
#include "../../util/db.h"
#include "../../util/dist_mat.h"
#include "../../util/log.h"
#include "../runner.h"

class SnapshotBuilder {
    DBClient m_db;
    std::shared_ptr<PathPlannerNode> m_planner;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::thread m_rosThread;

public:
    SnapshotBuilder() : m_db({DB_USER, DB_PASSWORD}) {}
    ~SnapshotBuilder() { shutdown(); }

    int run() {
        const auto rooms = m_db.rooms();
        if (!rooms.has_value()) {
            throw std::runtime_error("Could not load rooms from DB");
        }
        DES_LOG_INFO(rclcpp::get_logger("des.snapshot"), "Loaded %zu rooms from DB", rooms->size());
        for (const auto& [_, room] : rooms.value()) {
            DES_LOG_DEBUG_STREAM(rclcpp::get_logger("des.snapshot"), room);
        }

        m_planner = std::make_shared<PathPlannerNode>(rooms.value());
        if (!m_planner->isReady()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.snapshot"), "Nav2 planner not available — is planner.sh running?");
            return 1;
        }

        m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        m_rosThread = std::thread([this] {
            m_executor->add_node(m_planner);
            m_executor->spin();
            m_executor->remove_node(m_planner);
        });

        // Matrix index order follows the (name-sorted) RoomMap iteration.
        std::vector<des::Room> locations;
        locations.reserve(rooms->size());
        for (const auto& [_, loc] : rooms.value()) {
            locations.push_back(loc);
        }

        const bool ok = DistMat::rebuild(locations, m_planner);
        shutdown();
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
