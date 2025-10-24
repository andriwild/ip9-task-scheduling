#pragma once

#include "../util/util.h"
#include "robot.h"
#include "../datastructure/graph.h"
#include "../datastructure/tree.h"
#include "../model/event.h"
#include "../util/tree_composer.h"

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
        for (const int id: searchIds) {
            Tree<SimulationEvent> tmpTree;
            auto tmpRoot = tmpTree.createRoot(std::make_unique<Tour>(currentTimeOffset, "Search"));
            currentTimeOffset = tour(tmpRoot, currentPos, id);
            root->addSubtree(tmpTree.releaseRoot());
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
        root->addSubtree(tmpTree.releaseRoot());
        return currentTimeOffset + duration;

    }

    Tree<SimulationEvent> escortSequence(
        int meetingTime,
        util::PersonData personData,
        int personId,
        int meetingLocation,
        int startLocation,
        int dock
    ) {

        std::string pId = "P" + std::to_string(personId);
        Tree<SimulationEvent> meetingTree;
        auto meetingRoot = meetingTree.createRoot(std::make_unique<MeetingEvent>(meetingTime, meetingLocation, personId, "Meeting: " + pId));

        Tree<SimulationEvent> escortTree;
        auto escortTreeRoot = escortTree.createRoot(std::make_unique<EscortEvent>(0, personId, "Escort"));
        auto escortTour = escortTreeRoot->addChild(std::make_unique<Tour>(0, "Tour"));
        tour(escortTour, personData[personId].back(), meetingLocation);

        Tree<SimulationEvent> searchPersonTree;
        auto searchRoot = searchPersonTree.createRoot(std::make_unique<SearchEvent>(0, personId, "Search"));
        searchPerson(searchRoot, startLocation, personData[personId] );

        Tree<SimulationEvent> conversationTree;
        auto convRoot = conversationTree.createRoot(std::make_unique<TalkingEvent>(0, personId, "Conversation 1"));
        talkSequence(convRoot, 10, personId);

        Tree<SimulationEvent> conversationTree2;
        auto convRoot2 = conversationTree2.createRoot(std::make_unique<TalkingEvent>(0, personId, "Conversation 2"));
        talkSequence(convRoot2, 10, personId);

        Tree<SimulationEvent> dockTree;
        auto dockTreeRoot = dockTree.createRoot(std::make_unique<Tour>(0, "Dock"));
        tour(dockTreeRoot, meetingLocation, dock);

        EventTreeComposer etc{};
        etc.on(meetingRoot)
        ->prepend(conversationTree)
        ->prepend(escortTree)
        ->prepend(conversationTree2)
        ->prepend(searchPersonTree)
        ->append(dockTree);
        return meetingTree;
    }
};
