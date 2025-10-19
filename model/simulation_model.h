#pragma once
#include <QObject>

#include "event.h"
#include "robot.h"
#include "../datastructure/event_queue.h"
#include "../datastructure/graph.h"

class  SimulationModel final : public QObject {
    Q_OBJECT
    Graph &m_graph;
    Robot &m_robot;
    EventQueue &m_events;

public:
    explicit SimulationModel(Graph &graph, Robot &robot, EventQueue &events):
    m_graph(graph),
    m_robot(robot),
    m_events(events)
    {}

    void step() {
        const auto event = m_events.getNextEvent();
        event->execute(m_events);
        emit robotChanged(m_robot);
        emit timeChanged(event->getTime());
   }

    Graph& getGraph() const { return m_graph; };

    Robot& getRobot() const { return m_robot; }


signals:
    void graphChanged(Graph& graph);
    void robotChanged(Robot &robot);
    void timeChanged(int time);
};
