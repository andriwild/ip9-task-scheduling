#pragma once

#include "../datastructure/tree.h"
#include "../model/event.h"
#include "../util/tree_composer.h"

class Planner {
    double m_speed;

public:
    explicit Planner(const double speed): m_speed(speed) {}

     int tour(EV::TreeNode<IEvent> *root, const int from, const int to) const {
        return 0;
    }

    int searchPerson(EV::TreeNode<IEvent> *root, const int from, const std::vector<int> &searchIds ) const {
        int currentPos = from;
        int currentTimeOffset = root->getValue()->getTime();
        for (const int id: searchIds) {
            EV::Tree<IEvent> tmpTree;
            auto tmpRoot = tmpTree.createRoot(std::make_unique<Tour>(currentTimeOffset, "Search"));
            currentTimeOffset = tour(tmpRoot, currentPos, id);
            root->addSubtree(tmpTree.releaseRoot());
            currentPos = id;
        }
        return currentTimeOffset;
    }

    int talkSequence(EV::TreeNode<IEvent> *root, const int duration, int personId) {
        int currentTimeOffset = root->getValue()->getTime();
        EV::Tree<IEvent> tmpTree;
        auto tmpRoot = tmpTree.createRoot(std::make_unique<TalkingEvent>(currentTimeOffset, personId));
        tmpRoot->addChild(std::make_unique<TalkingEventStart>(currentTimeOffset, personId, "Hello"));
        tmpRoot->addChild(std::make_unique<TalkingEventEnd>(currentTimeOffset + duration, personId, "Bye"));
        root->addSubtree(tmpTree.releaseRoot());
        return currentTimeOffset + duration;
    }

    int bufferSequence(EV::TreeNode<IEvent> *root, const int duration) {
        int currentTimeOffset = root->getValue()->getTime();
        root->addChild(std::make_unique<BufferStartEvent>(currentTimeOffset, "BS"));
        root->addChild(std::make_unique<BufferEndEvent>(currentTimeOffset + duration, "BE"));
        return currentTimeOffset + duration;
    }

    EV::Tree<IEvent> escortSequence(
        int meetingTime,
        int personId,
        int meetingLocation,
        int startLocation,
        int dock
    ) {

        std::string pId = "P" + std::to_string(personId);
        EV::Tree<IEvent> meetingTree;
        auto meetingRoot = meetingTree.createRoot(std::make_unique<MeetingEvent>(meetingTime, meetingLocation, personId, "Meeting: " + pId));

        EV::Tree<IEvent> escortTree;
        auto escortTreeRoot = escortTree.createRoot(std::make_unique<EscortEvent>(0, personId, "Escort"));
        auto escortTour = escortTreeRoot->addChild(std::make_unique<Tour>(0, "Tour"));

        EV::Tree<IEvent> searchPersonTree;
        auto searchRoot = searchPersonTree.createRoot(std::make_unique<SearchEvent>(0, personId, "Search"));

        EV::Tree<IEvent> conversationTree;
        auto convRoot = conversationTree.createRoot(std::make_unique<TalkingEvent>(0, personId, "Conversation 1"));
        talkSequence(convRoot, 10, personId);

        EV::Tree<IEvent> conversationTree2;
        auto convRoot2 = conversationTree2.createRoot(std::make_unique<TalkingEvent>(0, personId, "Conversation 2"));
        talkSequence(convRoot2, 10, personId);

        EV::Tree<IEvent> dockTree;
        auto dockTreeRoot = dockTree.createRoot(std::make_unique<Tour>(0, "Dock"));
        tour(dockTreeRoot, meetingLocation, dock);

        EV::Tree<IEvent> bufferTree;
        auto bufferTreeRoot = bufferTree.createRoot(std::make_unique<BufferEvent>(0, "B"));
        bufferSequence(bufferTreeRoot, 100);

        EventTreeComposer etc{};
        etc.on(meetingRoot)
        ->prepend(conversationTree)
        ->prepend(escortTree)
        ->prepend(conversationTree2)
        ->prepend(bufferTree)
        ->prepend(searchPersonTree)
        ->append(dockTree);
        return meetingTree;
    }
};
