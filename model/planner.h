#pragma once

#include "../util/util.h"
#include "robot.h"
#include "../datastructure/graph.h"
#include "../datastructure/tree.h"
#include "../model/event.h"

class Planner {
    Graph &m_graph;
    double m_speed;

public:
    explicit Planner(Graph &graph, const double speed): m_graph(graph), m_speed(speed) {}

     int tour(TreeNode<SimulationEvent> *root, const int from, const int to) const {
        const std::vector<int> path = m_graph.dijkstra(from).shortestPath(to);
        const std::vector<int> driveTimes = util::calcDriveTime(path, m_graph, m_speed);

        // from goal to start
        int currentTime = root->getValue()->getTime();
        for (int pathNode = 1; pathNode < path.size(); pathNode++) {
            const int driveTime = driveTimes[pathNode-1];
            root->addChild(
                std::make_unique<RobotDriveStartEvent>(
                currentTime,
                path[pathNode],
                DRIVE)
                );
            root->addChild(std::make_unique<RobotDriveEndEvent>(currentTime + driveTime - 1, path[pathNode]));
            currentTime += driveTime;
        }
        return currentTime;
    }

    int searchPerson(TreeNode<SimulationEvent> *root, const int from, const std::vector<int> &searchIds ) const {
        int currentPos = from;
        int currentTimeOffset = root->getValue()->getTime();
        int searchTourCounter = 0;
        for (const int id: searchIds) {
            Tree<SimulationEvent> tmpTree;
            auto tmpRoot = tmpTree.createRoot(std::make_unique<Tour>(currentTimeOffset, "Search tour " + std::to_string(searchTourCounter++)));
            currentTimeOffset = tour(tmpRoot, currentPos, id);
            root->addSubtree(tmpTree.getRoot());
            currentPos = id;
        }
        return currentTimeOffset;
    }

    int talkSequence(TreeNode<SimulationEvent> *root, const int duration, int personId) {
        int currentTimeOffset = root->getValue()->getTime();
        Tree<SimulationEvent> tmpTree;
        auto tmpRoot = tmpTree.createRoot(std::make_unique<TalkingEvent>(currentTimeOffset, personId));
        tmpRoot->addChild(std::make_unique<TalkingEventStart>(currentTimeOffset, personId, "Hello"));
        tmpRoot->addChild(std::make_unique<TalkingEventEnd>(currentTimeOffset + duration, personId, "Bye"));
        root->addSubtree(tmpTree.getRoot());
        return currentTimeOffset + duration;
    }
};
