#include <behaviortree_cpp/bt_factory.h>

#include "../../model/robot.h"

class RobotState final : public BT::SyncActionNode {
    Robot* m_robot;

public:
    explicit RobotState(const std::string& name, const BT::NodeConfig& config, Robot* robot):
    SyncActionNode(name, config),
    m_robot(robot)
    {}

    static BT::PortsList providedPorts() {
        return { BT::BidirectionalPort<int>("robot_task") };
    }

    BT::NodeStatus tick() override {
        int currentState;
        getInput<int>("robot_task", currentState);
        m_robot->setTask(static_cast<ROBOT_STATE>(currentState));
        return BT::NodeStatus::SUCCESS;
    }
};