#pragma once
#include "behaviortree_cpp/action_node.h"
#include "../../datastructure/tree.h"
//#include "../../model/event.h"
#include "../../des/event.h"
#include "../../model/sim_time.h"


class EventHandler : public BT::SyncActionNode {
    EV::Tree<IEvent>* m_eventTree;
    std::vector<EV::TreeNode<IEvent>*> m_events;
    ReadOnlyClock* m_clock;
    std::shared_ptr<des::BaseEvent> m_nextEvent;
    int index = 0;
    EV::TreeNode<IEvent>* nextEvent;
    BT::Blackboard* m_bb;

public:
    EventHandler(
        const std::string &name,
        const BT::NodeConfig &config,
        EV::Tree<IEvent> *events,
        ReadOnlyClock* clock
    )
        : SyncActionNode(name, config),
          m_eventTree(events),
          m_clock(clock) {

        m_bb = config.blackboard.get();

        m_events = m_eventTree->traversalPreOrder();
        std::sort(m_events.begin(), m_events.end(),
            [](const EV::TreeNode<IEvent>* ev1, const EV::TreeNode<IEvent>* ev2) {
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

    BT::NodeStatus tick() override {
        //std::cout << "[ EventHandler ] tick\n";
        const auto event = nextEvent->getValue();

        const int t = m_clock->getTime();

        // has next event?
        if (m_events.size() > index) {
            const int timeDiff = event->getTime() - t;
            // timeDiff = 0: nothing to do
            // timeDiff > 0: we are early
            // timeDiff < 0: we are late

            if (timeDiff > 0) {
                // check leftMost child of the current subtask (nothing to do)
                // next event buffer? replan and consume buffer
                // if not, replan: shift events to now
            } else if (timeDiff < 0) {
                // delete events? time buffer events, search points?
                // replan: shrink buffers, remove events
            } else {
                // timeDiff == 0
                // next event == buffer, consume it
            }

            event->execute(*m_bb);
            //std::cout << "Execute event: " << nextEvent->getValue()->getId() << ", index:" << index << "\n";

            if (m_events.size() > index) {
                nextEvent = m_events.at(index++);
            }
        }

        return BT::NodeStatus::SUCCESS;
    }
};
