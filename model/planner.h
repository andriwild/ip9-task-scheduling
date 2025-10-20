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

    int driveSequence(Robot &robot, int startTime, const int from, const int to, ROBOT_TASK task) {
        std::vector<int> path = m_graph.dijkstra(from).shortestPath(to);
        std::vector<int> driveTimes = util::calcDriveTime(path, m_graph, robot.getSpeed());

        // from goal to start
        int currentTime = startTime;
        for (int nodeId = 1; nodeId < path.size(); nodeId++) {
            int driveTime = driveTimes[nodeId-1];
            m_events.addEvent<RobotDriveStartEvent>(
                &robot,
                currentTime,
                currentTime + driveTime,
                path[nodeId],
                task
            );
            m_events.addEvent<RobotDriveEndEvent>(&robot, currentTime + driveTime - 1, path[nodeId]);
            currentTime += driveTime;
        }
        return currentTime;
    }

    int driveSequenceReverse(Robot &robot, int goalArrivalTime, const int from, const int to, ROBOT_TASK task) {
        std::vector<int> path = m_graph.dijkstra(from).shortestPath(to);
        std::vector<int> driveTimes = util::calcDriveTime(path, m_graph, robot.getSpeed());

        // from goal to start
        for (int nodeId = path.size() -1; nodeId > 0; nodeId--) {
            m_events.addEvent<RobotDriveStartEvent>(
                &robot,
                goalArrivalTime - driveTimes[nodeId -1],
                goalArrivalTime,
                path[nodeId],
                task
            );
            m_events.addEvent<RobotDriveEndEvent>(&robot, goalArrivalTime-1, path[nodeId]);
            goalArrivalTime -= driveTimes[nodeId - 1];
        }
        return goalArrivalTime;
    }

    void addEscortEvents(Robot &robot, MeetingEvent &ev, std::vector<int> searchLocations) {
        // from goal back to start
        int dest = ev.getDestination();

        int timeOffset = ev.getTime();
        int lastSearchLocation = searchLocations.back();
        int firstSearchLocation = searchLocations.front();
        timeOffset = driveSequenceReverse(robot, timeOffset, lastSearchLocation, dest, ESCORT);

        searchLocations.pop_back();
        std::ranges::reverse(searchLocations);
        int to = lastSearchLocation;
        for (auto searchLocation: searchLocations) {
            const int from = searchLocation;
            timeOffset = driveSequenceReverse(robot, timeOffset, from, to, SEARCH);
            to = from;
        }
        driveSequenceReverse(robot, timeOffset, robot.getPosition(), firstSearchLocation, DRIVE);
    }


    void process(Robot &robot, util::PersonData personData) {
        for(auto ev: m_events.getAllEvents()) {
            if (auto* escortEvent = dynamic_cast<MeetingEvent*>(ev)) {
                auto searchLocations = personData.at(escortEvent->getPersonId());
                addEscortEvents(robot, *escortEvent, searchLocations);
                driveSequence(robot, escortEvent->getTime() ,escortEvent->getDestination(), robot.getDock(), DRIVE);
            }
        }
    }
};