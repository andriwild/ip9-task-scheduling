#pragma once

#include <memory>
#include <QString>
#include <QColor>
#include "../../event_system_core/src/util/types.h"

struct VisualEvent {
    int time;
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

struct RobotStateMeta {
    int index;
    QColor color;
    std::string label;
};


inline RobotStateMeta getMeta(int type) {
    switch (static_cast<des::RobotStateType>(type)) {
        case des::RobotStateType::IDLE:
            return {0, Qt::lightGray, "Idle" };
        case des::RobotStateType::MOVING:
            return {1, QColor(100, 200, 100), "Moving" };
        case des::RobotStateType::ACCOMPANY:
            return {2, QColor(200, 150, 50), "Accompany" };
        case des::RobotStateType::CHARGING:
            return {3, Qt::yellow, "Charging" };
        case des::RobotStateType::SEARCHING:
            return {4, QColor(200, 100, 100), "Searching" };
        case des::RobotStateType::CONVERSATE:
            return {5, QColor(180, 215, 230), "Conversate" };
    }
};
