#pragma once

#include "engine/contracts/i_event.h"
#include "person_accompany_event.h"
#include "engine/event/start_drive_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot_state.h"
#include "plugins/accompany/accompany_order.h"
#include "plugins/accompany/states.h"
#include "model/room.h"

namespace des {

class StartAccompanyEvent final : public IEvent {
    OrderPtr m_order;
public:
    explicit StartAccompanyEvent(const int time, const OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartAccompanyEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        auto& accompany = static_cast<AccompanyOrder&>(*m_order);
        // For the paper: The robot hands the person over on the spot instead of walking them  to the appointment room.
        if (ctx.getConfig()->searchDropOffAtFind) {
            accompany.roomName = ctx.getRobot()->getLocation();
        }
        const auto& personName = accompany.personName;
        if (ctx.hasEmployee(personName)) {
            auto person = ctx.getPersonByName(personName);
            const std::string currentRoom = ctx.getPersonLocation(personName);
            ctx.pushEvent(std::make_shared<PersonAccompanyDepartureEvent>(time, person, currentRoom));
        }
        ctx.changeRobotState(std::make_unique<AccompanyState>());
        static_cast<AccompanyOrder&>(*m_order).phase = AccompanyPhase::ACCOMPANY;

        const Room& currentRoom = ctx.room(ctx.getRobot()->getLocation());
        auto roomDrive = std::make_shared<StartDriveEvent>(time, std::make_shared<RoomTarget>(accompany.roomName));
        requestDrive(ctx, currentRoom.m_waypoint, currentRoom.m_tour.visibilityAt(0), roomDrive);

        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Start Accompany"; }
    EventType getType() const override { return EventType::START_ACCOMPANY; }
};

}  // namespace des
