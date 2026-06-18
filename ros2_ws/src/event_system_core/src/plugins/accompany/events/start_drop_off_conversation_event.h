#pragma once

#include <cassert>

#include "model/event/base.h"
#include "drop_off_conversation_complete.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"
#include "plugins/accompany/accompany_plugin.h"
#include "plugins/accompany/states.h"
#include "util/rnd.h"

class StartDropOffConversationEvent final : public IEvent {
public:
    explicit StartDropOffConversationEvent(const int time) : IEvent(time) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartDropOffConversationEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        assert(!ctx.getRobot()->isDriving());
        double eventTime = this->time + rndAccompanyConversationTime(ctx.rng());
        if (rnd::uni(ctx.rng()) < accompanyConfig().conversationProbability) {
            ctx.startActivity(std::make_shared<SuccessDropOffConversationCompleteEvent>(eventTime));
        } else {
            ctx.startActivity(std::make_shared<FailedDropOffConversationCompleteEvent>(eventTime));
        }
        ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Start Drop Off Conversation"; }
    des::EventType getType() const override { return des::EventType::START_DROP_OFF_CONV; }
};
