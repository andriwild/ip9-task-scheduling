#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "../../model/robot.h"
#include "../../model/sim_time.h"
#include "../../util/util.h"

class NavigateToPoint final : public BT::StatefulActionNode {
public:

    NavigateToPoint(
        const std::string& name,
        const BT::NodeConfig& config,
        Robot* r,
        ReadOnlyClock* clock
    ) : StatefulActionNode(name, config),
        m_goalNode(0),
        m_arrivalTime(0),
        m_robot(r),
        m_clock(clock) {
        std::cout << "Navigate to point start at time: " << m_clock->getTime() << std::endl;
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("goal") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    int m_goalNode;
    int m_arrivalTime;
    Robot* m_robot;
    ReadOnlyClock* m_clock;
};

inline BT::NodeStatus NavigateToPoint::onStart() {
    if ( !getInput<int>("goal", m_goalNode)) {
        std::cout << " [ Robot start ] abort - no goal\n";
        return BT::NodeStatus::FAILURE;
    }
    //std::cout << "[ Robot start ] - Goal Node: " << std::to_string(m_goalNode) << std::endl;

    // TODO: nav2 drive time
    // m_arrivalTime = m_clock->getTime() + driveTime;
    return BT::NodeStatus::RUNNING;
}

inline BT::NodeStatus NavigateToPoint::onRunning() {

    const int timeRemaining = m_arrivalTime - m_clock->getTime();
    const std::string arriveTime = std::to_string(timeRemaining);
    if (timeRemaining <= 0) {
        //std::cout << "[ Robot at goal ] " << std::to_string(m_goalNode) << std::endl;
        m_robot->setPosition(m_goalNode);
        return BT::NodeStatus::SUCCESS;
    }
    //std::cout << "[ Robot is driving ... ] - Time remaining: " << arriveTime << " - " << m_clock->getTime() << std::endl;
    return BT::NodeStatus::RUNNING;
}

inline void NavigateToPoint::onHalted() {
    std::cout << "[ Robot stopped ]" << std::endl;
}