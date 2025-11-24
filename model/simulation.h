#pragma once
#include <QObject>

#include "behaviortree_cpp/bt_factory.h"
#include "event.h"
#include "robot.h"
#include "sim_time.h"
#include "../datastructure/tree.h"

class  Simulation final : public QObject {
    Q_OBJECT
    Robot* m_robot;
    EV::Tree<IEvent> &m_events;
    std::vector<EV::TreeNode<IEvent>*> m_allEvents;
    BT::Tree &m_tree;
    SimTime *m_simTime;

public:
    explicit Simulation(
        Robot *robot,
        EV::Tree<IEvent> &events,
        BT::Tree &tree,
        SimTime* simTime
    ) : 
        m_robot(robot),
        m_events(events),
        m_tree(tree),
        m_simTime(simTime)
    {}

    void simStep() {
        m_tree.tickOnce();
        m_simTime->inc();
        emit robotChanged(m_robot);
        emit timeChanged(m_simTime->getTime());

        // if (m_allEvents.empty()) {
        //     m_allEvents = m_events.traversalPreOrder();
        // }
        // const auto event = m_allEvents.front();
        // m_allEvents.erase(m_allEvents.begin());
        // SimulationEvent* ev = event->getValue();
        // ev->execute(m_robot, m_events);
    }

    Robot* getRobot() const { return m_robot; }
    EV::Tree<IEvent>& getEvents() const { return m_events; }

signals:
    void robotChanged(Robot *robot);
    void timeChanged(int time);
    void eventsChanged(EV::Tree<IEvent> &events);
};
