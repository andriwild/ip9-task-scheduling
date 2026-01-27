#pragma once

#include <memory>
#include <QString>
#include <QColor>
#include "../../event_system_core/src/util/types.h"
#include "../../event_system_core/src/model/robot_state.h"

struct VisualEvent {
    int time;
    QString label;
    uint8_t type;
};

struct VisualStateBlock {
    int startTime;
    int endTime;
    int type;
};

struct VisualAppointment {
    std::shared_ptr<des::Appointment> appt;
    int startTime;
};

struct RobotStateMeta {
    int index;
    QColor color;
    std::string label;
    std::string labelShort;
};


inline RobotStateMeta getMeta(int type) {
    switch (static_cast<RobotStateType>(type)) {
        case RobotStateType::IDLE:
            return {0, Qt::lightGray, "Idle", "i" };
        case RobotStateType::MOVING:
            return {1, QColor(100, 200, 100), "Moving", "m" };
        case RobotStateType::ACCOMPANY:
            return {2, QColor(200, 150, 50), "Accompany", "acc" };
        case RobotStateType::CHARGING:
            return {3, Qt::yellow, "Charging", "c" };
        case RobotStateType::SEARCHING:
            return {4, QColor(200, 100, 100), "Searching", "s" };
        case RobotStateType::CONVERSATE:
            return {5, QColor(180, 215, 230), "Converate", "conv" };
    }
};
