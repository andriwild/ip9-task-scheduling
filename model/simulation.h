#pragma once
#include <QObject>

#include "event.h"
#include "robot.h"
#include "../datastructure/event_queue.h"
#include "../datastructure/graph.h"

class  Simulation final : public QObject {
    Q_OBJECT
    Graph &m_graph;
    Robot &m_robot;
    EventQueue &m_events;
    util::PersonData &m_personData;

public:
    explicit Simulation(Graph &graph, Robot &robot, EventQueue &events, util::PersonData &personData):
    m_graph(graph),
    m_robot(robot),
    m_events(events),
    m_personData(personData)
    {}

   void simStep() {
        const auto event = m_events.getNextEvent();
        event->execute(m_events);
        emit robotChanged(m_robot);
        emit timeChanged(event->getTime());
   }

    Graph& getGraph() const { return m_graph; };
    Robot& getRobot() const { return m_robot; }
    EventQueue& getEvents() const { return m_events; };
    util::PersonData& getPersonData() const { return m_personData; };

signals:
    void graphChanged(Graph& graph);
    void robotChanged(Robot &robot);
    void timeChanged(int time);
    void eventsChanged(EventQueue &events);
    void personChanged(util::PersonData &peronData);
};
