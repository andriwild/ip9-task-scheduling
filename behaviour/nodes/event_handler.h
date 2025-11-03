#pragma once
#include "behaviortree_cpp/action_node.h"
#include "../../datastructure/tree.h"
//#include "../../model/event.h"
#include "../../des/event.h"
#include "../../model/sim_time.h"


struct EventComparator {
    bool operator()(const std::shared_ptr<des::BaseEvent>& a,
                    const std::shared_ptr<des::BaseEvent>& b) const {
        // min heap
        return a->getTime() > b->getTime();
    }
};

typedef std::priority_queue<
        std::shared_ptr<des::BaseEvent>,
        std::vector<std::shared_ptr<des::BaseEvent> >,
        EventComparator
    > EventQueue;



class EventHandler : public BT::SyncActionNode {
    EV::Tree<SimulationEvent>* m_eventTree;
    std::vector<EV::TreeNode<SimulationEvent>*> m_events;
    ReadOnlyClock* m_clock;
    EventQueue queue{};
    std::shared_ptr<des::BaseEvent> m_nextEvent;
    int index = 0;
    EV::TreeNode<SimulationEvent>* nextEvent;
    BT::Blackboard* bb;



    void printQueue(EventQueue queue) {
        while (!queue.empty()) {
            auto& event = queue.top();
            std::cout << "Event ID: " << event->getId()
                      << ", Time: " << event->getTime() << "\n";
            queue.pop();
        }
    }


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
        // auto me = std::make_shared<des::MeetingEvent>(400,0);
        // auto de1 = std::make_shared<des::DriveEvent>(20, 0, 1, DRIVE, me.get());
        // auto de2 = std::make_shared<des::DriveEvent>(130, 0, 1, DRIVE, me.get());
        // auto de3 = std::make_shared<des::DriveEvent>(230, 0, 1, DRIVE, me.get());
        //
        // queue.push(me);
        // queue.push(de1);
        // queue.push(de2);
        // queue.push(de3);
        // printQueue(queue);
        //
        // m_nextEvent = queue.top();
        // queue.pop();

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

        int t = m_clock->getTime();

        while (t >= nextEvent->getValue()->getTime()) {

            nextEvent->getValue()->execute(*bb);
            //std::cout << "Execute event: " << nextEvent->getValue()->getId() << ", index:" << index << "\n";
            nextEvent = m_events.at(index++);

        }


        // get time
        //  - roboter was driving 100
        // I am late? -> replanning
        // next event
        // set bb (event.execute(bb))



        // std::cout << "[ Event Handler ]\n";
        //
        // std::vector<int> queue;
        // if (getInput<std::vector<int>>("queue", queue)) {
        //     if (!queue.empty()) {
        //         std::cout << "queue is not empty\n";
        //         setOutput("goal", queue.back());
        //         std::cout << "set new goal: " << queue.back() << "\n";
        //         queue.pop_back();
        //         setOutput("queue", queue);
        //         return BT::NodeStatus::SUCCESS;
        //     }
        //
        // }
        return BT::NodeStatus::SUCCESS;
    }
};