#pragma once
#include <QObject>

#include "behaviortree_cpp/bt_factory.h"
#include "event.h"
#include "robot.h"

class  Simulation final : public QObject {
    Q_OBJECT
    Robot* m_robot;
    BT::Tree &m_tree;

public:
    explicit Simulation(
        Robot *robot,
        BT::Tree &tree
    ): 
        m_robot(robot),
        m_tree(tree)
    {}

    void simStep() {
        m_tree.tickOnce();

        // if (m_allEvents.empty()) {
        //     m_allEvents = m_events.traversalPreOrder();
        // }
        // const auto event = m_allEvents.front();
        // m_allEvents.erase(m_allEvents.begin());
        // SimulationEvent* ev = event->getValue();
        // ev->execute(m_robot, m_events);
    }

    Robot* getRobot() const { return m_robot; }

signals:
    void robotChanged(Robot *robot);
    void timeChanged(int time);
};
