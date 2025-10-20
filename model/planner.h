#pragma once

#include "../util/util.h"
#include "robot.h"
#include "../datastructure/event_queue.h"
#include "../datastructure/graph.h"

namespace planner {
    static void addSearchEvents(
        Graph &graph,
        Robot &robot,
        EventQueue &events,
        const util::PersonData &personData)
    {
        for(auto ev: events.getAllEvents()) {
            if (auto* escortEvent = dynamic_cast<PersonEscortEvent*>(ev)) {
                int dest = escortEvent->getDestination();
                auto result = graph.dijkstra(robot.getPosition());
                std::vector<int> path = result.shortestPath(dest);
                std::vector<int> driveTimes = util::calcDriveTime(path, graph, robot.getSpeed());
                int maxTime = std::accumulate(driveTimes.begin(), driveTimes.end(), 0);

                int currentTime = escortEvent->getTime() - maxTime;
                for (int i = 0; i < path.size() - 1; i++) {
                    events.addEvent<RobotDriveStartEvent>(&robot, currentTime, path[i]);
                    currentTime = currentTime + driveTimes[i];
                    events.addEvent<RobotDriveEndEvent>(&robot, currentTime, path[i+1]);
                }
            }
        }
    }
};
