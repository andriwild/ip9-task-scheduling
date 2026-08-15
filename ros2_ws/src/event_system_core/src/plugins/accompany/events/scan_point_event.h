#pragma once

#include "engine/contracts/i_event.h"
#include "engine/contracts/i_sim_context.h"
#include "model/person.h"
#include "model/robot.h"
#include "plugins/accompany/accompany_order.h"
#include "util/geometry.h"
#include "util/log.h"

namespace des {

constexpr double kIdentificationMargin = 0.99;

class ScanPointEvent final : public IEvent {
    OrderPtr m_order;

public:
    // Filled on execution, read by the room count of the metrics reporter.
    std::string m_room;

    explicit ScanPointEvent(const int time, const OrderPtr& order)
        : IEvent(time), m_order(order) {}

    std::shared_ptr<IEvent> withTime(int newTime) const override {
        auto copy = std::make_shared<ScanPointEvent>(*this);
        copy->time = newTime;
        copy->cancelled = false;
        return copy;
    }

    // Someone the robot perceives but cannot tell apart from here, and has not walked
    // up to during this mission. Returns the first one, the rest wait for the next scan.
    static std::string nextRecognizedPerson(const ISimContext& ctx, const AccompanyOrder& order) {
        for (const auto& [name, room] : ctx.getAllPersonLocations()) {
            if (order.identified.contains(name) || order.detoured.contains(name)) {
                continue;
            }
            const Person* person = ctx.getPersonByName(name);
            if (person != nullptr && person->busy) {
                continue;
            }
            if (ctx.robotRecognizesPerson(name) && !ctx.robotSeesPerson(name)) {
                return name;
            }
        }
        return {};
    }

    // Walk up to the recognized person and come back to the point we are standing on.
    // They lie inside the current visibility polygon, so the way there is free and the same polygon serves for the identification
    static void insertDetour(ISimContext& ctx, AccompanyOrder& order, const std::string& recognizedPerson) {
        const auto target = ctx.getPersonPosition(recognizedPerson);
        if (!target) {
            return;
        }
        const auto robot = ctx.getRobot();
        const ScanStop here{ robot->getPosition(), robot->getVisibility(), true };
        const Point stop = geom::approachPoint(*target, here.point, kIdentificationMargin * ctx.getConfig()->personIdentificationRange);
        order.detoured.insert(recognizedPerson);
        order.scanQueue.push_front(here);
        order.scanQueue.push_front(ScanStop{ stop, robot->getVisibility(), true });
        DES_LOG_DEBUG("des.plugin.accompany.search", "Detour to %s in %s, %zu stops queued", recognizedPerson.c_str(), robot->getLocation().c_str(), order.scanQueue.size());
    }

    void execute(ISimContext& ctx) override {
        auto& accompany = static_cast<AccompanyOrder&>(*m_order);
        const auto& personName = accompany.personName;
        m_room = ctx.getRobot()->getLocation();
        const bool seen = ctx.robotSeesPerson(personName);
        const bool asks = !seen && ctx.getConfig()->personDirectionsProbability > 0.0;

        for (const auto& [name, room] : ctx.getAllPersonLocations()) {
            if (name == personName || !ctx.robotSeesPerson(name) || !accompany.identified.insert(name).second) {
                continue;
            }
            if (asks) {
                accompany.pendingAsk.push_back(name);
                DES_LOG_DEBUG("des.plugin.accompany.search", "Queued %s to be asked about %s", name.c_str(), personName.c_str());
            }
        }

        if (!seen) {
            const std::string recognizedPerson = nextRecognizedPerson(ctx, accompany);
            if (!recognizedPerson.empty()) {
                insertDetour(ctx, accompany, recognizedPerson);
            }
        }

        DES_LOG_DEBUG("des.plugin.accompany.search", "ScanPoint t=%d room=%s stops_left=%zu person=%s seen=%d", this->time, ctx.getRobot()->getLocation().c_str(), accompany.scanQueue.size(), personName.c_str(), seen);

        if (seen) {
            ctx.getRobot()->setIsPersonVisible(true);
            if (auto* person = ctx.getPersonByName(personName)) {
                person->busy = true;
            }
        }
        ctx.notifyEvent(*this);
        ctx.tickBT();
    }

    std::string getName() const override { return "Scan"; }
    EventType getType() const override { return EventType::SCAN; }
};

}  // namespace des
