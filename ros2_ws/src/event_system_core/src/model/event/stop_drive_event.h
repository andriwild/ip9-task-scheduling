#pragma once

#include "base.h"
#include "person_transition.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../../plugins/accompany/accompany_order.h"

class StopDriveEvent final : public IEvent {
public:
    double distance;
    std::string location;

    explicit StopDriveEvent(const int time, const std::string &location, const double distance)
        : IEvent(time)
        , distance(distance)
        , location(location)
    {}

    void execute(ISimContext& ctx) override {
        ctx.robotMoved(this->location, this->distance);
        ctx.getRobot()->setDriving(false);
        ctx.setBTBlackboard("location", this->location);
        ctx.notifyEvent(*this);

        // Move accompanied person to arrival location
        auto accompany = std::dynamic_pointer_cast<AccompanyOrder>(ctx.getOrderPtr());
        if (ctx.getRobot()->getStateType() == des::RobotStateType::ACCOMPANY && accompany) {
            const auto& personName = accompany->personName;
            if (ctx.hasEmployee(personName)) {
                auto person = ctx.getPersonByName(personName);
                const auto& arrivalLocation = ctx.getRobot()->getLocation();
                ctx.setPersonLocation(personName, arrivalLocation);
                auto transition = std::make_shared<PersonTransitionEvent>(this->time, person);
                transition->targetRoom = arrivalLocation;
                ctx.pushEvent(transition);
            }
        }

        ctx.tickBT();
    }

    std::string getName() const override { return "Arrived: " + location; }
    des::EventType getType() const override { return des::EventType::STOP_DRIVE; }
};
