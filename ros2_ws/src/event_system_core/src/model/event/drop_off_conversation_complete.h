#pragma once

#include "base.h"
#include "../i_sim_context.h"
#include "../robot.h"
#include "../robot_state.h"

class DropOffConversationCompleteEvent : public IEvent {
public:
    explicit DropOffConversationCompleteEvent(const int time) : IEvent(time) {}
    std::string getName() const override { return "Conversation complete"; }
    des::EventType getType() const override { return des::EventType::DROP_OFF_CONV_COMPLETE; }
};

class SuccessDropOffConversationCompleteEvent final : public DropOffConversationCompleteEvent {
public:
    explicit SuccessDropOffConversationCompleteEvent(const int time)
        : DropOffConversationCompleteEvent(time)
    {}

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->getState()->setResult(des::Result::SUCCESS);
        ctx.getRobot()->setIsPersonVisible(false);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Conversation Successful"; }
};

class FailedDropOffConversationCompleteEvent final : public DropOffConversationCompleteEvent {
public:
    explicit FailedDropOffConversationCompleteEvent(const int time)
        : DropOffConversationCompleteEvent(time)
    {}

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->getState()->setResult(des::Result::FAILURE);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Conversation Failed "; }
};
