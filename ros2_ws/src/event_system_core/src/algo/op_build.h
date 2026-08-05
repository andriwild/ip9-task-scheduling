/*
 * Helpers to put a route planning problem together.
 * Start and end places become nodes without reward or cost,
 * and the distance matrix is filled from the path planner.
 *
 */

#pragma once

#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "op_types.h"
#include "engine/contracts/i_path_planning.h"
#include "model/order.h"
#include "../util/log.h"

constexpr double kMinEnergyBudgetWh = 1e-3;

struct OpBudgets {
    float timeBudget;
    float energyBudget;
    float initialSoc;
    float endSocMin;
    float socThreshold;
    float maxEnergy;
    float chargeTimePerWh;
    float chargeTimePerWhTapered;
    float cvEnergy;
};

namespace op_build {

struct PlannedNode {
    OpNode op;
    des::OrderPtr order;
};

inline PlannedNode anchorNode(const std::string& location) {
    return { OpNode{ location, 0.0f, 0.0f, 0.0f }, nullptr };
}

inline std::optional<Mat> distanceMatrix(const IPathPlanning& paths, const std::vector<PlannedNode>& planned) {
    const size_t n = planned.size();
    Mat mat(n, std::vector<float>(n, 0.0f));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            const auto d = paths.getDistance(planned[i].op.name, planned[j].op.name);
            if (!d) {
                DES_LOG_ERROR(rclcpp::get_logger("des.algo.op"), "No distance %s -> %s", planned[i].op.name.c_str(), planned[j].op.name.c_str());
                return std::nullopt;
            }
            mat[i][j] = static_cast<float>(*d);
            mat[j][i] = mat[i][j];
        }
    }
    return mat;
}

}  // namespace op_build
