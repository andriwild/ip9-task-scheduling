#pragma once

#include <cassert>

#include "base.h"
#include "drop_off_conversation_complete.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../robot_state.h"
#include "../../util/rnd.h"

class StartDropOffConversationEvent final : public IEvent {
public:
    explicit StartDropOffConversationEvent(const int time) : IEvent(time) {}

    void execute(ISimContext& ctx) override {
        assert(!ctx.getRobot()->isDriving());
        double eventTime = this->time + ctx.getRndConversationTime();
        if (rnd::uni(ctx.rng()) < ctx.getConversationProbability()) {
            ctx.pushEvent(std::make_shared<SuccessDropOffConversationCompleteEvent>(eventTime));
        } else {
            ctx.pushEvent(std::make_shared<FailedDropOffConversationCompleteEvent>(eventTime));
        }
        ctx.changeRobotState(std::make_unique<ConversateState>(ConversateState::Type::DROP_OFF));
        ctx.notifyEvent(*this);
    }

    std::string getName() const override { return "Start Drop Off Conversation"; }
    des::EventType getType() const override { return des::EventType::START_DROP_OFF_CONV; }
};
