#pragma once

#include <cassert>
#include <memory>
#include <string>
#include <utility>

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/robot.h"
#include "model/robot_state.h"
#include "plugins/order_registry.h"
#include "util/rnd.h"
#include "util/types.h"
#include "model/person.h"

namespace des {

struct ConversationSpec {
    ConversationKind kind = ConversationKind::FOUND_PERSON;
    std::string partner;
    double durationMean = 0.0;
    double durationStd = 0.0;
    double successProbability = 1.0;
};

class EndConversationEvent final : public IEvent {
    ConversationSpec m_spec;
    bool m_success;

public:
    EndConversationEvent(const int time, ConversationSpec spec, const bool success)
        : IEvent(time), m_spec(std::move(spec)), m_success(success) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<EndConversationEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        ctx.getRobot()->getState()->setResult(m_success ? Result::SUCCESS : Result::FAILURE);
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    ConversationKind getKind() const { return m_spec.kind; }
    bool wasSuccessful() const { return m_success; }

    std::string getName() const override {
        return m_success ? "Conversation Successful" : "Conversation Failed ";
    }

    EventType getType() const override { return EventType::CONVERSATION_END; }
};

class StartConversationEvent final : public IEvent {
    ConversationSpec m_spec;

public:
    StartConversationEvent(const int time, ConversationSpec spec)
        : IEvent(time), m_spec(std::move(spec)) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<StartConversationEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    void execute(ISimContext& ctx) override {
        assert(!ctx.getRobot()->isDriving());
        const double sampled = rnd::normal(ctx.robotRng(), m_spec.durationMean, m_spec.durationStd);
        const double eventTime = this->time + (sampled < 1.0 ? 1.0 : sampled);
        const bool success = rnd::uni(ctx.robotRng()) < m_spec.successProbability;

        ctx.startActivity(std::make_shared<EndConversationEvent>(static_cast<int>(eventTime), m_spec, success));
        ctx.changeRobotState(std::make_unique<ConversationState>(m_spec.kind));
        if (const auto order = ctx.getOrderPtr()) {
            OrderRegistry::instance().get(order->type).onConversationStart(ctx, *order, m_spec.kind);
        }
        ctx.notifyEvent(*this);
    }

    ConversationKind getKind() const { return m_spec.kind; }

    std::string getName() const override {
        switch (m_spec.kind) {
            case ConversationKind::FOUND_PERSON: {
                return "Found Person Conversation";
            }
            case ConversationKind::DROP_OFF: {
                return "Start Drop Off Conversation";
            }
            case ConversationKind::ASK_DIRECTIONS: {
                return "Ask For Directions";
            }
        }
        return "Conversation";
    }

    EventType getType() const override { return EventType::CONVERSATION_START; }
};

}  // namespace des
