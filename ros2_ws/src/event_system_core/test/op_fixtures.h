#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "../src/algo/op.h"

namespace op_fixtures {

inline des::Mat uniformMatrix(const std::size_t size, const float distance) {
    des::Mat mat(size, std::vector<float>(size, distance));
    for (std::size_t i = 0; i < size; ++i) {
        mat[i][i] = 0.0f;
    }
    return mat;
}

inline des::Mat lineMatrix(const std::vector<float>& position) {
    des::Mat mat(position.size(), std::vector<float>(position.size(), 0.0f));
    for (std::size_t i = 0; i < position.size(); ++i) {
        for (std::size_t j = 0; j < position.size(); ++j) {
            mat[i][j] = std::fabs(position[i] - position[j]);
        }
    }
    return mat;
}

inline des::OpInstance twoTasks(const float serviceTimeA, const float serviceTimeB) {
    std::vector<des::OpNode> nodes = {
        des::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::OpNode{"A", 1.0f, serviceTimeA, 0.0f},
        des::OpNode{"B", 1.0f, serviceTimeB, 0.0f},
    };
    des::Mat mat = uniformMatrix(nodes.size(), 10.0f);
    const des::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = 1000.0f,
        .energyBudget = 1000.0f,
        .initialSoc   = 1000.0f,
    };
    return des::OpInstance(std::move(nodes), std::move(mat), {}, params);
}

inline des::OpInstance slowVersusHungry(const float timeBudget, const float energyBudget) {
    std::vector<des::OpNode> nodes = {
        des::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::OpNode{"slow", 1.0f, 100.0f, 1.0f},
        des::OpNode{"hungry", 1.0f, 1.0f, 100.0f},
    };
    des::Mat mat = uniformMatrix(nodes.size(), 10.0f);
    const des::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = timeBudget,
        .energyBudget = energyBudget,
        .initialSoc   = 100000.0f,
    };
    return des::OpInstance(std::move(nodes), std::move(mat), {}, params);
}

inline des::OpInstance lineInstance(const float timeBudget) {
    std::vector<des::OpNode> nodes = {
        des::OpNode{"start", 0.0f, 0.0f, 0.0f},
        des::OpNode{"end", 0.0f, 0.0f, 0.0f},
        des::OpNode{"A", 1.0f, 10.0f, 0.0f},
        des::OpNode{"B", 4.0f, 10.0f, 0.0f},
        des::OpNode{"C", 1.0f, 20.0f, 0.0f},
    };
    des::Mat mat = lineMatrix({ 0.0f, 40.0f, 10.0f, 20.0f, 30.0f });
    const des::OpParams params {
        .startNodeId  = 0,
        .endNodeId    = 1,
        .timeBudget   = timeBudget,
        .energyBudget = 1000.0f,
        .initialSoc   = 1000.0f,
    };
    return des::OpInstance(std::move(nodes), std::move(mat), {}, params);
}

}  // namespace op_fixtures
