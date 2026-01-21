#pragma once
#include <iomanip>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/loggers/abstract_logger.h"

class SimpleLogger : public BT::StatusChangeLogger {
    std::map<uint16_t, int> depth_map;

public:
    SimpleLogger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode()) {
        calculateDepth(tree.rootNode(), 0);
    }

    void flush() override {
        // std::cout << std::flush;
    }

    void callback(BT::Duration timestamp, const BT::TreeNode& node,
                  BT::NodeStatus prev_status, BT::NodeStatus status) override {
        if (prev_status == BT::NodeStatus::IDLE && status == BT::NodeStatus::IDLE) {
            return;
        }

        double t = std::chrono::duration<double>(timestamp).count();

        std::string name = node.name();
        if (name.empty()) {
            name = node.registrationName();
        }

        int indentLevel = depth_map[node.UID()];
        std::string indent(indentLevel * 2, ' ');  // 2 spaces per level

        // Using stream to format the message
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << "[" << t << "] "
           << indent
           << std::left << std::setw(30 - indent.length()) << name
           << " : " << toStr(prev_status)
           << " -> " << toStr(status);

        RCLCPP_INFO(rclcpp::get_logger("BTLogger"), "%s", ss.str().c_str());
    }

private:
    void calculateDepth(const BT::TreeNode * node, int depth) {
        if (!node) {
            return;
        }

        depth_map[node->UID()] = depth;

        if (auto control = dynamic_cast<const BT::ControlNode *>(node)) {
            for (const auto& child : control->children()) {
                calculateDepth(child, depth + 1);
            }
        } else if (auto decorator = dynamic_cast<const BT::DecoratorNode *>(node)) {
            if (decorator->child()) {
                calculateDepth(decorator->child(), depth + 1);
            }
        }
    }
};
