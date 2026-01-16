#pragma once

#include <memory>
#include <QString>
#include <QColor>
#include "../../event_system_core/src/util/types.h"
#include "../../event_system_core/src/model/robot_state.h"

struct VisualEvent {
    int time;
    QString label;
    QString type;
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


inline QColor getColor(int type) {
    switch (static_cast<RobotStateType>(type)) {
        case RobotStateType::IDLE:
            return Qt::lightGray;
        case RobotStateType::MOVING:
            return QColor(100, 200, 100);
        case RobotStateType::ACCOMPANY:
            return QColor(200, 150, 50);
        case RobotStateType::CHARGING:
            return Qt::yellow;
        case RobotStateType::SEARCHING:
            return QColor(200, 100, 100);
        case RobotStateType::CONVERSATE:
            return QColor(180, 215, 230);
        default:
            return Qt::gray;
    }
};

inline int getYPos(int type) {
    switch (static_cast<RobotStateType>(type)) {
        case RobotStateType::IDLE:
            return 0;
        case RobotStateType::MOVING:
            return 10;
        case RobotStateType::ACCOMPANY:
            return 20;
        case RobotStateType::CHARGING:
            return 30;
        case RobotStateType::SEARCHING:
            return 40;
        case RobotStateType::CONVERSATE:
            return 50;
        default:
            return 0;
    }
};

