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
    std::shared_ptr<des::Appointment> appt;
    int startTime;
};

struct TimelineTransformer {
    double pixelsPerSecond;
    int simStartTime;
    double xOffset;

    double toX(int time) const {
        return xOffset + (time - simStartTime) * pixelsPerSecond;
    }

    double toWidth(int duration) const {
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
        case des::EventType::ABORT_SEARCH:
            return QColor(220, 0, 0);

        case des::EventType::START_DROP_OFF_CONV:
        case des::EventType::DROP_OFF_CONV_COMPLETE:
        case des::EventType::START_FOUND_PERSON_CONV:
        case des::EventType::FOUND_PERSON_CONV_COMPLETE:
            return QColor(130, 0, 200);

        case des::EventType::START_ACCOMPANY:
            return QColor(0, 100, 255);

        case des::EventType::STOP_DRIVE:
        case des::EventType::MISSION_COMPLETE:
            return QColor(0, 150, 50);

        default: return Qt::black;
    }
}


inline RobotStateMeta getMeta(int type) {
    switch (static_cast<des::RobotStateType>(type)) {
        case des::RobotStateType::IDLE:
            return {0, QColor(200, 200, 200), "Idle"};
        case des::RobotStateType::SEARCHING:
            return {4, QColor(255, 100, 100), "Searching"};
        case des::RobotStateType::CONVERSATE:
            return {5, QColor(180, 130, 220), "Conversation"};
        case des::RobotStateType::ACCOMPANY:
            return {2, QColor(100, 180, 255), "Accompany"};
        // case des::RobotStateType::MOVING:
        //     return {1, QColor(80, 200, 120), "Moving"};
        case des::RobotStateType::CHARGING:
            return {3, QColor(255, 210, 50), "Charging"};
        default: return {-1, Qt::gray, "Unknown"};
    }
}
