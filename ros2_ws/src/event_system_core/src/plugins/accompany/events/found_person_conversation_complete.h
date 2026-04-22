#pragma once

#include "model/event/base.h"
#include "model/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"

class FoundPersonConversationCompleteEvent : public IEvent {
public:
    explicit FoundPersonConversationCompleteEvent(const int time) : IEvent(time) {}
    std::string getName() const override { return "Conversation complete"; }
    des::EventType getType() const override { return des::EventType::FOUND_PERSON_CONV_COMPLETE; }
};

class SuccessFoundPersonConversationCompleteEvent final : public FoundPersonConversationCompleteEvent {
public:
    explicit SuccessFoundPersonConversationCompleteEvent(const int time)
        : FoundPersonConversationCompleteEvent(time)
    {}

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->getState()->setResult(des::Result::SUCCESS);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Conversation Successful"; }
};

class FailedFoundPersonConversationCompleteEvent final : public FoundPersonConversationCompleteEvent {
public:
    explicit FailedFoundPersonConversationCompleteEvent(const int time)
        : FoundPersonConversationCompleteEvent(time)
    {}

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->getState()->setResult(des::Result::FAILURE);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Conversation Failed "; }
};
