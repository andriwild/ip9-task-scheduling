#pragma once
#include "../model/robot.h"
#include "../view/helper.h"
#include "behaviortree_cpp/blackboard.h"


namespace des {
    inline static int nextId;
    class BaseEvent {
        int m_id;
        int m_time;

    public:
        virtual ~BaseEvent() = default;

        explicit BaseEvent(const int time):
        m_id(nextId++),
        m_time(time)
        {}

        int getId() const { return m_id; }
        int getTime() const { return m_time; };

        virtual void execute(BT::Blackboard& bb) = 0;

        bool operator<(const BaseEvent& other) const {
            return m_time < other.m_time;
        }
    };


    class MeetingEvent final : public BaseEvent {
        int m_personId;
        BaseEvent* m_parent;

    public:
        explicit MeetingEvent(const int time, int personId, BaseEvent* parent = nullptr):
        BaseEvent(time),
        m_personId(personId),
        m_parent(parent)
        {}

        void execute(BT::Blackboard &bb) override {
            std::cout << "( Meeting Event )\n";
        }
    };


    class DriveEvent final : public BaseEvent {
        int m_from, m_to;
        ROBOT_STATE m_type;
        BaseEvent* m_parent;

    public:
        explicit DriveEvent(const int time, const int from, const int to, const ROBOT_STATE type = ROBOT_STATE::DRIVE, BaseEvent* parent = nullptr):
        BaseEvent(time),
        m_from(from),
        m_to(to),
        m_type(type),
        m_parent(parent)
        {}

        void execute(BT::Blackboard &bb) override {
            std::cout << "( Drive Event ) - Goal: " << m_to << ", Type: " << Helper::toString(m_type) << "\n";
            bb.set("goal", m_to);
            bb.set("type", m_type);
        }
    };
}