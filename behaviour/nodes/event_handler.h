#pragma once
#include "behaviortree_cpp/action_node.h"
#include "../../datastructure/tree.h"
#include "../../model/event.h"
#include "../../model/sim_time.h"


struct CompareEvent {
    bool operator()(const SimulationEvent* e1, const SimulationEvent* e2) {
        return e1->getTime() > e2->getTime();
    }
};

class EventHandler : public BT::SyncActionNode {
    EV::Tree<SimulationEvent>* m_eventTree;
    std::vector<EV::TreeNode<SimulationEvent>*> m_events;
    std::priority_queue<
        SimulationEvent*,
        std::vector<SimulationEvent*>,
        CompareEvent
    > queue;
    int index = 0;
    ReadOnlyClock* m_clock;
    SimulationEvent* currentEvent;

    EV::TreeNode<SimulationEvent> *getNext() {
        auto ev = m_events.at(index);
        if (index < m_events.size()) {
            index ++;
        }
        return ev;
    }

public:
    EventHandler(
        const std::string &name,
        const BT::NodeConfig &config,
        EV::Tree<SimulationEvent> *events,
        ReadOnlyClock* clock
    )
        : SyncActionNode(name, config), m_eventTree(events),
          m_clock(clock)
    {

        for (const auto ev: events->traversalPreOrder()) {
            queue.emplace(ev->getValue());
        }
        currentEvent = queue.top();
        std::cout << currentEvent->getTime() << " " << currentEvent->getName() << std::endl;
    }

    static BT::PortsList providedPorts() {
        return { };
    }

    BT::NodeStatus tick() {
        int t = m_clock->getTime();
        std::cout << "[ Event Handler ] " << t << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};