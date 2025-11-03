#pragma once
#include <behaviortree_cpp/bt_factory.h>

class DriveStart : public BT::SyncActionNode {
    int m_pos;
public:
    DriveStart(const std::string& name, const BT::NodeConfig& config):
    SyncActionNode(name, config),
    m_pos(0)
    {}

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<int>("goal") };
    }

    BT::NodeStatus tick() override {
        if (true) {
            setOutput("goal", 10);
        }
        done = true;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool done = false;
};


class SayHello : public BT::SyncActionNode {
    std::string m_text;
    int counter;
public:
    SayHello(const std::string &name, const BT::NodeConfig& config):
    SyncActionNode(name, config),
    counter(0)
    {}

    BT::NodeStatus tick() override {
        counter ++;
        setOutput("text", m_text + " " + std::to_string(counter) );
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<std::string>("text") };
    }
};


class LogNode: public BT::SyncActionNode {
    std::string m_text;
public:
    LogNode(const std::string &name, const BT::NodeConfig& config):
    SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {

        if ( getInput<std::string>("msg", m_text)) {
            std::cout << m_text <<"\n";
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("msg") };
    }
};

