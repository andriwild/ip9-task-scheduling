#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "i_path_planner.h"

// Answers distance queries from a precomputed matrix instead of Nav2. No ROS.
class MatrixPlanner : public IPathPlanner {
    std::vector<std::vector<float>> m_mat;
    std::unordered_map<std::string, int> m_index;

public:
    MatrixPlanner(const std::vector<std::string>& names, std::vector<std::vector<float>> mat)
        : m_mat(std::move(mat)) {
        for (int i = 0; i < static_cast<int>(names.size()); ++i) {
            m_index.emplace(names[i], i);
        }
    }

    std::optional<double> calcDistance(const std::string& from, const std::string& to, bool) override {
        const auto f = m_index.find(from);
        const auto t = m_index.find(to);
        if (f == m_index.end() || t == m_index.end()) {
            return std::nullopt;
        }
        return m_mat[f->second][t->second];
    }
};
