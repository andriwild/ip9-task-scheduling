#pragma once

#include <memory>
#include <QString>
#include <QColor>
#include "../../event_system_core/src/util/types.h"

struct VisualEvent {
    QString label;
    des::EventType type;
};

struct VisualStateBlock {
    int startTime;
    int endTime;
    uint8_t type;
};

struct VisualAppointment {
    int id;
    int scheduledTime;
    int startTime;
    std::string orderType;
    std::string primaryLabel;
    std::string description;
};

struct TimelineTransformer {
    double pixelsPerSecond;
    int simStartTime;
    double xOffset;

    double toX(const int time) const {
        return xOffset + (time - simStartTime) * pixelsPerSecond;
    }

    double toWidth(const int duration) const {
        return duration * pixelsPerSecond;
    }
};

struct RobotStateMeta {
    int index;
    QColor color;
    std::string label;
};

inline QColor getEventColor(des::EventType type) {
    switch (type) {
        case des::EventType::MISSION_DISPATCH:
        case des::EventType::MISSION_COMPLETE:
            return QColor(220, 0, 0);

        case des::EventType::ABORT_SEARCH:
            return QColor(255, 165, 0);

        case des::EventType::START_DROP_OFF_CONV:
        case des::EventType::DROP_OFF_CONV_COMPLETE:
        case des::EventType::START_FOUND_PERSON_CONV:
        case des::EventType::FOUND_PERSON_CONV_COMPLETE:
            return QColor(130, 0, 200);

        case des::EventType::START_ACCOMPANY:
            return QColor(0, 100, 255);

        case des::EventType::INFORMATION:
        case des::EventType::INFORMATION_START:
            return QColor(0, 140, 155);

        case des::EventType::STOP_DRIVE:
        case des::EventType::START_DRIVE:
            return QColor(0, 150, 50);

        default: return Qt::black;
    }
}


// Look up rendering metadata. `name` (plugin-supplied) wins when set so each
// mission flavour stays distinct on the timeline; the structural category is
// the fallback when no name is known (e.g. legacy/unknown messages).
// Keep this palette in sync with des_swimlane_panel.cpp::stateColor.
inline RobotStateMeta getMeta(int category, const std::string& name = "") {
    if (name == "search")     return {4, QColor(255, 100, 100), "Searching"};
    if (name == "conversate") return {5, QColor(180, 130, 220), "Conversation"};
    if (name == "accompany")  return {2, QColor(100, 180, 255), "Accompany"};
    if (name == "clean")      return {7, QColor(240, 150, 200), "Cleaning"};
    if (name == "acquire")    return {8, QColor(220, 180, 100), "Acquiring"};
    switch (static_cast<des::RobotStateType>(category)) {
        case des::RobotStateType::IDLE:
            return {0, QColor(200, 200, 200), "Idle"};
        case des::RobotStateType::CHARGING:
            return {3, QColor(255, 210, 50), "Charging"};
        case des::RobotStateType::RETURNING:
            return {6, QColor(120, 200, 160), "Returning"};
        case des::RobotStateType::MISSION:
            return {9, QColor(170, 170, 220), "Mission"};
        default: return {-1, Qt::gray, "Unknown"};
    }
}
