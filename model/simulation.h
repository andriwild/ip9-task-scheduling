#pragma once
#include <QObject>

#include "behaviortree_cpp/bt_factory.h"
#include "event.h"
#include "robot.h"
#include "sim_time.h"
#include "../datastructure/graph.h"
#include "../datastructure/tree.h"
#include "../behaviour/nodes/nav_to_point.h"

class  Simulation final : public QObject {
    Q_OBJECT
    Graph &m_graph;
    Robot* m_robot;
    EV::Tree<SimulationEvent> &m_events;
    util::PersonData &m_personData;
    std::vector<EV::TreeNode<SimulationEvent>*> m_allEvents;
    BT::Tree &m_tree;
    SimTime *m_simTime;

public:
    explicit Simulation(
        Graph &graph,
        Robot *robot,
        EV::Tree<SimulationEvent> &events,
        util::PersonData &personData,
        BT::Tree &tree,
        SimTime* simTime
    ) : m_graph(graph),
        m_robot(robot),
        m_events(events),
        m_personData(personData),
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

    Graph& getGraph() const { return m_graph; }
    Robot* getRobot() const { return m_robot; }
    EV::Tree<SimulationEvent>& getEvents() const { return m_events; }
    util::PersonData& getPersonData() const { return m_personData; }

signals:
    void graphChanged(Graph& graph);
    void robotChanged(Robot *robot);
    void timeChanged(int time);
    void eventsChanged(EV::Tree<SimulationEvent> &events);
    void personChanged(util::PersonData &peronData);
};
