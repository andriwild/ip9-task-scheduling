#pragma once
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include <iostream>
#include <iomanip>
#include <map>

class SimpleLogger : public BT::StatusChangeLogger {
    // Map speichert die Tiefe (Einrückung) für jeden Node (via UID)
    std::map<uint16_t, int> depth_map;

public:
    SimpleLogger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode()) {
        // Tiefe aller Nodes einmalig berechnen
        calculateDepth(tree.rootNode(), 0);
    }

    // Wichtig: Override von flush(), damit der Buffer geleert wird
    void flush() override {
        std::cout << std::flush;
    }

    void callback(BT::Duration timestamp, const BT::TreeNode& node, 
                  BT::NodeStatus prev_status, BT::NodeStatus status) override 
    {
        // Filter: IDLE-Wechsel interessieren uns meist nicht
        if (prev_status == BT::NodeStatus::IDLE && status == BT::NodeStatus::IDLE) return;

        // Zeit (Sekunden.Millisekunden)
        double t = std::chrono::duration<double>(timestamp).count();
        
        // Farbe wählen
        const char* colorCode = "\033[0m"; // Reset
        if (status == BT::NodeStatus::SUCCESS)      colorCode = "\033[32m"; // Grün
        else if (status == BT::NodeStatus::FAILURE) colorCode = "\033[31m"; // Rot
        else if (status == BT::NodeStatus::RUNNING) colorCode = "\033[33m"; // Gelb
        else if (status == BT::NodeStatus::IDLE)    colorCode = "\033[90m"; // Grau

        // Name ermitteln
        std::string name = node.name();
        if (name.empty()) name = node.registrationName();

        // Einrückung ermitteln
        int indentLevel = depth_map[node.UID()];
        std::string indent(indentLevel * 2, ' '); // 2 Leerzeichen pro Ebene

        std::cout << std::fixed << std::setprecision(3) << "[" << t << "] "
                  << indent 
                  << std::left << std::setw(30 - indent.length()) << name 
                  << " : " << toStr(prev_status) 
                  << " -> " << colorCode << toStr(status) << "\033[0m" 
                  << std::endl;
                  
        // Optional: Sofort flushen, falls du Schritt-für-Schritt debuggst
        // flush();
    }

private:
    // Rekursive Funktion zum Aufbau der Tiefen-Map
    void calculateDepth(const BT::TreeNode* node, int depth) {
        if (!node) return;
        
        depth_map[node->UID()] = depth;

        // Falls es ein ControlNode (Sequence, Fallback) oder Decorator ist, hat er Kinder
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
