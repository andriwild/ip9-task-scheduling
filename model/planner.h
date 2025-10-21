#pragma once

#include "../util/util.h"
#include "robot.h"
#include "../datastructure/event_queue.h"
#include "../datastructure/graph.h"
#include "../model/event.h"

class Planner {
    Graph &m_graph;
    EventQueue &m_events;

public:
    explicit Planner(Graph &graph, EventQueue &events):
    m_graph(graph),
    m_events(events)
    {}

    int driveSequence(int startTime, const int from, const int to, double speed, ROBOT_TASK task) {
        std::vector<int> path = m_graph.dijkstra(from).shortestPath(to);
        std::vector<int> driveTimes = util::calcDriveTime(path, m_graph, speed);

        // from goal to start
        int currentTime = startTime;
        for (int nodeId = 1; nodeId < path.size(); nodeId++) {
            int driveTime = driveTimes[nodeId-1];
            m_events.addEvent<RobotDriveStartEvent>(
                currentTime,
                currentTime + driveTime-1,
                path[nodeId],
                task
            );
            //m_events.addEvent<RobotDriveEndEvent>(&robot, currentTime + driveTime - 1, path[nodeId]);
            currentTime += driveTime;
        }
        return currentTime;
    }

    int driveSequenceReverse(int goalArrivalTime, const int from, const int to, double speed, ROBOT_TASK task) {
        std::vector<int> path = m_graph.dijkstra(from).shortestPath(to);
        std::vector<int> driveTimes = util::calcDriveTime(path, m_graph, speed);

        // from goal to start
        for (int nodeId = path.size() -1; nodeId > 0; nodeId--) {
            m_events.addEvent<RobotDriveStartEvent>(
                goalArrivalTime - driveTimes[nodeId -1] +1,
                goalArrivalTime,
                path[nodeId],
                task
            );
            //m_events.addEvent<RobotDriveEndEvent>(&robot, goalArrivalTime-1, path[nodeId]);
            goalArrivalTime -= driveTimes[nodeId - 1];
        }
        return goalArrivalTime;
    }

    void addEscortEvents(MeetingEvent &ev, std::vector<int> searchLocations, int startNode, double speed) {
        // from goal back to start
        int dest = ev.getDestination();

        int timeOffset = ev.getTime();
        int lastSearchLocation = searchLocations.back();
        int firstSearchLocation = searchLocations.front();
        timeOffset = driveSequenceReverse(timeOffset, lastSearchLocation, dest, speed, ESCORT);

        searchLocations.pop_back();
        std::ranges::reverse(searchLocations);
        int to = lastSearchLocation;
        for (auto searchLocation: searchLocations) {
            const int from = searchLocation;
            timeOffset = driveSequenceReverse(timeOffset, from, to, speed, SEARCH);
            to = from;
        }
        driveSequenceReverse(timeOffset, startNode, firstSearchLocation, speed, DRIVE);
    }


    void process(int startNodeId, util::PersonData personData, double speed) {
        for(auto ev: m_events.getAllEvents()) {
            if (auto* escortEvent = dynamic_cast<MeetingEvent*>(ev)) {
                auto searchLocations = personData.at(escortEvent->getPersonId());
                addEscortEvents(*escortEvent, searchLocations, startNodeId, speed);
            }
        }
    }

    Tree<SimulationEvent> tour(int timeOffset, int from, int to, double speed) {
        std::vector<int> path = m_graph.dijkstra(from).shortestPath(to);
        std::vector<int> driveTimes = util::calcDriveTime(path, m_graph, speed);

        Tree<SimulationEvent> eventTree;
        auto root = eventTree.createRoot(std::make_unique<Tour>());

        // from goal to start
        int currentTime = timeOffset;
        for (int nodeId = 1; nodeId < path.size(); nodeId++) {
            int driveTime = driveTimes[nodeId-1];
            root->addChild(
                std::make_unique<RobotDriveStartEvent>(
                currentTime,
                currentTime + driveTime-1,
                path[nodeId],
                DRIVE)
                );
            root->addChild(std::make_unique<RobotDriveEndEvent>(currentTime + driveTime - 1, path[nodeId]));
            currentTime += driveTime;
        }
        return eventTree;
    }
};