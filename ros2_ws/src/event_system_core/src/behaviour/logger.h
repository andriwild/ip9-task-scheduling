#pragma once
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include <iostream>
#include <iomanip>
#include <map>

class SimpleLogger : public BT::StatusChangeLogger {
    std::map<uint16_t, int> depth_map;

public:
    SimpleLogger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode()) {
        calculateDepth(tree.rootNode(), 0);
    }

    void flush() override {
        std::cout << std::flush;
    }

    void callback(BT::Duration timestamp, const BT::TreeNode& node, 
                  BT::NodeStatus prev_status, BT::NodeStatus status) override 
    {
        if (prev_status == BT::NodeStatus::IDLE && status == BT::NodeStatus::IDLE) return;

        double t = std::chrono::duration<double>(timestamp).count();
        
        const char* colorCode = "\033[0m"; // Reset
        if (status == BT::NodeStatus::SUCCESS)      colorCode = "\033[32m"; // Grün
        else if (status == BT::NodeStatus::FAILURE) colorCode = "\033[31m"; // Rot
        else if (status == BT::NodeStatus::RUNNING) colorCode = "\033[33m"; // Gelb
        else if (status == BT::NodeStatus::IDLE)    colorCode = "\033[90m"; // Grau

        std::string name = node.name();
        if (name.empty()) name = node.registrationName();

        int indentLevel = depth_map[node.UID()];
        std::string indent(indentLevel * 2, ' '); // 2 Leerzeichen pro Ebene

        std::cout << std::fixed << std::setprecision(3) << "[" << t << "] "
                  << indent 
                  << std::left << std::setw(30 - indent.length()) << name 
                  << " : " << toStr(prev_status) 
                  << " -> " << colorCode << toStr(status) << "\033[0m" 
                  << std::endl;
    }

private:
    void calculateDepth(const BT::TreeNode* node, int depth) {
        if (!node) return;
        
        depth_map[node->UID()] = depth;

        if (auto control = dynamic_cast<const BT::ControlNode*>(node)) {
            for (const auto& child : control->children()) {
                calculateDepth(child, depth + 1);
            }
        }
        else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node)) {
            if (decorator->child()) {
                calculateDepth(decorator->child(), depth + 1);
            }
        }
    }
};
