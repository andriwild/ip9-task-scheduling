#pragma once

#include <memory>
#include <QString>
#include "../../event_system_core/src/util/types.h"

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

