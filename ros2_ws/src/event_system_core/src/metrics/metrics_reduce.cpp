#include "metrics/metrics_reduce.h"

#include <algorithm>

#include "plugins/charge/charge_order.h"
#include "plugins/order_registry.h"
#include "util/constants.h"

namespace des::reduce {

namespace {

int duration(const StateInterval& in) {
    return in.endTime - in.time;
}

double consumed(const StateInterval& in, const double capacity) {
    return (in.socFrom - in.socTo) * capacity;
}

using EventGroup   = std::vector<const IEvent*>;
using EventsByType = std::map<des::EventType, EventGroup>;

EventsByType groupByType(const EventList& protocol) {
    EventsByType out;
    for (const auto& event : protocol) {
        out[event->getType()].push_back(event.get());
    }
    return out;
}

const EventGroup& group(const EventsByType& byType, const des::EventType type) {
    static const EventGroup empty;
    const auto it = byType.find(type);
    return it == byType.end() ? empty : it->second;
}

void addOutcome(MissionTotals& out, const des::IOrder& order, const int time) {
    MissionStats& total = out.byMode[order.execution];
    MissionStats& day   = out.byDay[time / SECONDS_PER_DAY][order.execution];
    const int timeDiff  = time - order.deadline.value_or(time);

    switch (order.state) {
        case des::MissionState::COMPLETED:
            if (timeDiff > 0) {
                total.late++;
                day.late++;
                out.accLateTime += timeDiff;
                if (!out.hasLate || timeDiff < out.minLateness) {
                    out.minLateness = timeDiff;
                }
                if (!out.hasLate || timeDiff > out.maxLateness) {
                    out.maxLateness = timeDiff;
                }
                out.hasLate = true;
            } else {
                total.onTime++;
                day.onTime++;
                out.accEarlyTime += timeDiff;
            }
            break;
        case des::MissionState::CANCELLED:
            total.cancelled++;
            day.cancelled++;
            break;
        case des::MissionState::FAILED: {
            total.failed++;
            day.failed++;
            const std::string detail = OrderRegistry::instance().get(order.type).outcomeDetail(order);
            if (!detail.empty()) {
                day.failByDetail[detail]++;
                total.failByDetail[detail]++;
            }
            break;
        }
        case des::MissionState::REJECTED:
            total.rejected++;
            day.rejected++;
            break;
        default:
            break;
    }
}

}  // namespace

MissionTotals missions(const EventList& protocol) {
    const EventsByType byType = groupByType(protocol);
    MissionTotals out;

    out.chargeCyclesComplete = static_cast<int>(
        group(byType, des::EventType::BATTERY_FULL).size() +
        group(byType, des::EventType::CHARGE_MISSION).size());

    for (const auto type : { des::EventType::MISSION_DISPATCH,
                             des::EventType::ORDER_ARRIVAL,
                             des::EventType::BACKGROUND_RELEASE }) {
        for (const IEvent* event : group(byType, type)) {
            if (const auto order = event->getOrder()) {
                out.byMode[order->execution].registered++;
            }
        }
    }

    for (const IEvent* event : group(byType, des::EventType::MISSION_COMPLETE)) {
        const auto order = event->getOrder();
        if (!order || order->type == kChargeOrderType) {
            continue;
        }
        addOutcome(out, *order, event->time);
    }

    return out;
}

DriveTotals drives(const EventList& protocol) {
    DriveTotals out;
    int startedAt  = -1;
    for (const auto& e : protocol) {
        const auto type = e->getType();
        if (type == des::EventType::START_DRIVE) {
            startedAt = e->time;
            continue;
        }
        if (type != des::EventType::STOP_DRIVE) {
            continue;
        }
        if (startedAt >= 0) {
            out.movingTime += e->time - startedAt;
            startedAt = -1;
        }
        const double distance = e->getDistance();
        out.distance += distance;
        out.distanceByDay[e->time / SECONDS_PER_DAY] += distance;
    }
    return out;
}

std::map<int, int> chargeCyclesByDay(const std::vector<int>& chargeSessions) {
    std::map<int, int> out;
    for (const int time : chargeSessions) {
        out[time / SECONDS_PER_DAY]++;
    }
    return out;
}

StateTotals states(const StateLog& log, const double capacity) {
    StateTotals out;
    for (const auto& in : log.entries()) {
        const int span    = duration(in);
        const double ah   = consumed(in, capacity);
        out.timeByName[in.name]           += span;
        out.energyByName[in.name]         += ah;
        out.timeByCategory[in.category]   += span;
        out.energyByCategory[in.category] += ah;
        out.totalTime                     += span;
    }
    return out;
}

BatteryTotals battery(const StateLog& log, const std::vector<int>& chargeSessions, const double capacity, const double lowThreshold, const double dischargedAh) {
    BatteryTotals out;
    const double lowFraction = lowThreshold / 100.0;
    double dodSum = 0.0;
    int dodCount  = 0;
    double lastChargeEndSoc = -1.0;
    bool wasCharging = false;

    for (const auto& in : log.entries()) {
        const bool isCharging = in.name == "charging";
        if (isCharging && !wasCharging) {
            if (lastChargeEndSoc >= 0.0) {
                const double dod = lastChargeEndSoc - in.socFrom;
                if (dod > 0.0) {
                    dodSum += dod;
                    dodCount++;
                }
            }
        } else if (!isCharging && wasCharging) {
            lastChargeEndSoc = in.socFrom;
        }
        wasCharging = isCharging;

        if (in.socFrom >= lowFraction && in.socTo < lowFraction) {
            out.deepDischarge++;
        }
    }

    out.cyclesTotal          = static_cast<int>(chargeSessions.size());
    out.dischargedAh         = dischargedAh;
    out.avgDepthOfDischarge  = dodCount > 0 ? dodSum / dodCount : 0.0;
    out.equivalentFullCycles = capacity > 0.0 ? dischargedAh / capacity : 0.0;
    return out;
}

std::map<int, DaySpan> perDay(const StateLog& log, const double capacity) {
    std::map<int, DaySpan> out;
    for (const auto& in : log.entries()) {
        const int span = duration(in);
        if (span <= 0) {
            continue;
        }
        const double ah = consumed(in, capacity);
        int cursor = in.time;
        while (cursor < in.endTime) {
            const int dayIndex = cursor / SECONDS_PER_DAY;
            const int dayEnd   = std::min(in.endTime, (dayIndex + 1) * SECONDS_PER_DAY);
            const int slice    = dayEnd - cursor;
            DaySpan& day = out[dayIndex];
            day.totalTime += slice;
            day.energyAh  += ah * slice / span;
            if (in.category == des::RobotStateType::MISSION) {
                day.missionTime += slice;
            } else if (in.category == des::RobotStateType::CHARGING) {
                day.chargingTime += slice;
            }
            if (in.name == "idle") {
                day.idleTime += slice;
            }
            cursor = dayEnd;
        }
        DaySpan& endDay = out[in.endTime / SECONDS_PER_DAY];
        if (endDay.minSoc < 0.0 || in.socTo < endDay.minSoc) {
            endDay.minSoc = in.socTo;
        }
    }
    return out;
}

}  // namespace des::reduce
