#pragma once
#include <behaviortree_cpp/bt_factory.h>

class DriveStart : public BT::SyncActionNode {
public:
    DriveStart(const std::string name, const BT::NodeConfig config):
    SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<int>("goal") };
    }

    BT::NodeStatus tick() override {
        if (!done) {
            setOutput("goal", 10);
        }
        done = true;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool done = false;
};