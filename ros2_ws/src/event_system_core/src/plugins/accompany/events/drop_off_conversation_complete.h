#pragma once

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"

namespace des {

class DropOffConversationCompleteEvent : public IEvent {
public:
    explicit DropOffConversationCompleteEvent(const int time) : IEvent(time) {}
    std::string getName() const override { return "Conversation complete"; }
    EventType getType() const override { return EventType::DROP_OFF_CONV_COMPLETE; }
};

class SuccessDropOffConversationCompleteEvent final : public DropOffConversationCompleteEvent {
public:
    explicit SuccessDropOffConversationCompleteEvent(const int time)
        : DropOffConversationCompleteEvent(time)
    {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<SuccessDropOffConversationCompleteEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->getState()->setResult(Result::SUCCESS);
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

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<FailedDropOffConversationCompleteEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->getState()->setResult(Result::FAILURE);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Conversation Failed "; }
};

}  // namespace des
