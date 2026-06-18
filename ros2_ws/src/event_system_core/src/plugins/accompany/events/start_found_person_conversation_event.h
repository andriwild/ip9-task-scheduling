#pragma once

#include <cassert>

#include "model/event/base.h"
#include "found_person_conversation_complete.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"
#include "plugins/accompany/accompany_plugin.h"
#include "plugins/accompany/states.h"
#include "util/rnd.h"

class StartFoundPersonConversationEvent final : public IEvent {
public:
    explicit StartFoundPersonConversationEvent(const int time) : IEvent(time) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartFoundPersonConversationEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        assert(!ctx.getRobot()->isDriving());
        auto eventTime = this->time + rndAccompanyConversationTime(ctx.rng());
        if (rnd::uni(ctx.rng()) < accompanyConfig().conversationProbability) {
            ctx.startActivity(std::make_shared<SuccessFoundPersonConversationCompleteEvent>(eventTime));
        } else {
            ctx.startActivity(std::make_shared<FailedFoundPersonConversationCompleteEvent>(eventTime));
        }
        ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::FOUND_PERSON));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Found Person Conversation"; }
    des::EventType getType() const override { return des::EventType::START_FOUND_PERSON_CONV; }
};
