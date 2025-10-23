#pragma once
#include <QObject>

#include "event.h"
#include "robot.h"
#include "../datastructure/graph.h"
#include "../datastructure/tree.h"

class  Simulation final : public QObject {
    Q_OBJECT
    Graph &m_graph;
    Robot &m_robot;
    Tree<SimulationEvent> &m_events;
    util::PersonData &m_personData;
    std::vector<TreeNode<SimulationEvent>*> m_allEvents;

public:
    explicit Simulation(Graph &graph, Robot &robot, Tree<SimulationEvent> &events, util::PersonData &personData):
    m_graph(graph),
    m_robot(robot),
    m_events(events),
    m_personData(personData)
    {}

   void simStep() {
        if (m_allEvents.empty()) {
            m_allEvents = m_events.traversalPreOrder();
        }
        const auto event = m_allEvents.front();
        m_allEvents.erase(m_allEvents.begin());
        SimulationEvent* ev = event->getValue();
        ev->execute(m_robot, m_events);

        emit robotChanged(m_robot);
        emit timeChanged(ev->getTime());
   }

    Graph& getGraph() const { return m_graph; };
    Robot& getRobot() const { return m_robot; }
    Tree<SimulationEvent>& getEvents() const { return m_events; };
    util::PersonData& getPersonData() const { return m_personData; };

signals:
    void graphChanged(Graph& graph);
    void robotChanged(Robot &robot);
    void timeChanged(int time);
    void eventsChanged(Tree<SimulationEvent> &events);
    void personChanged(util::PersonData &peronData);
};
