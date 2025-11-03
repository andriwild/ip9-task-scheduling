#pragma once
#include "behaviortree_cpp/action_node.h"
#include "../../datastructure/tree.h"
//#include "../../model/event.h"
#include "../../des/event.h"
#include "../../model/sim_time.h"


class EventHandler : public BT::SyncActionNode {
    EV::Tree<SimulationEvent>* m_eventTree;
    std::vector<EV::TreeNode<SimulationEvent>*> m_events;
    ReadOnlyClock* m_clock;
    std::shared_ptr<des::BaseEvent> m_nextEvent;
    int index = 0;
    EV::TreeNode<SimulationEvent>* nextEvent;
    BT::Blackboard* bb;

public:
    EventHandler(
        const std::string &name,
        const BT::NodeConfig &config,
        EV::Tree<SimulationEvent> *events,
        ReadOnlyClock* clock
    )
        : SyncActionNode(name, config),
          m_eventTree(events),
          m_clock(clock) {

        bb = config.blackboard.get();

        m_events = m_eventTree->traversalPreOrder();
        std::sort(m_events.begin(), m_events.end(),
            [](const EV::TreeNode<SimulationEvent>* ev1, const EV::TreeNode<SimulationEvent>* ev2) {
                return ev1->getValue()->getTime() < ev2->getValue()->getTime();
            });

        // for (auto ev : m_events) {
        //     std::cout << ev->getValue()->getId() << " -> " << ev->getValue()->getTime() << "\n";
        // }

        nextEvent = m_events.at(index++);
    }

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<int>("goal"), BT::BidirectionalPort<std::vector<int>>("queue") };
    }

    BT::NodeStatus tick() {
        //std::cout << "[ EventHandler ] tick\n";

        const int t = m_clock->getTime();

        while (t >= nextEvent->getValue()->getTime()) {
            nextEvent->getValue()->execute(*bb);
            //std::cout << "Execute event: " << nextEvent->getValue()->getId() << ", index:" << index << "\n";
            nextEvent = m_events.at(index++);
        }

        return BT::NodeStatus::SUCCESS;
    }
};