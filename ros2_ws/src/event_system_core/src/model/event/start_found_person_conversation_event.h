#pragma once

#include <cassert>

#include "base.h"
#include "found_person_conversation_complete.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../robot_state.h"
#include "../../util/rnd.h"

class StartFoundPersonConversationEvent final : public IEvent {
public:
    explicit StartFoundPersonConversationEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        assert(!ctx.getRobot()->isDriving());
        auto eventTime = this->time + ctx.getRndConversationTime();
        if (rnd::uni(ctx.rng()) < ctx.getConversationProbability()) {
            ctx.pushEvent(std::make_shared<SuccessFoundPersonConversationCompleteEvent>(eventTime));
        } else {
            ctx.pushEvent(std::make_shared<FailedFoundPersonConversationCompleteEvent>(eventTime));
        }
        ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Found Person Conversation"; }
    des::EventType getType() const override { return des::EventType::START_FOUND_PERSON_CONV; }
};
