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

    int driveSequence(Robot &robot, int goalArrivalTime, const int from, const int to, ROBOT_TASK task) {
        std::vector<int> path = m_graph.dijkstra(from).shortestPath(to);
        std::vector<int> driveTimes = util::calcDriveTime(path, m_graph, robot.getSpeed());

        // from start to goal
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

    void escort(Robot &robot, std::vector<int> searchLocations) {
        // from goal back to start
        for(auto ev: m_events.getAllEvents()) {
            if (auto* escortEvent = dynamic_cast<MeetingEvent*>(ev)) {
                int dest = escortEvent->getDestination();

                int timeOffset = escortEvent->getTime();
                int lastSearchLocation = searchLocations.back();
                int firstSearchLocation = searchLocations.front();
                timeOffset = driveSequence(robot, timeOffset, lastSearchLocation, dest, ESCORT);

                searchLocations.pop_back();
                std::ranges::reverse(searchLocations);
                int to = lastSearchLocation;
                for (auto searchLocation: searchLocations) {
                    const int from = searchLocation;
                    timeOffset = driveSequence(robot, timeOffset, from, to, SEARCH);
                    to = from;
                }
                driveSequence(robot, timeOffset, robot.getPosition(), firstSearchLocation, DRIVE);
            }
        }
    }
};