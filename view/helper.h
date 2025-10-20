#pragma once
#include <qcolor.h>

#include "../model/robot.h"

namespace Helper {

    inline QColor taskColor(ROBOT_TASK task) {
        QColor color = Qt::black;
        switch (task) {
            case SEARCH: color = Qt::cyan;
                break;
            case ESCORT: color = Qt::darkGreen;
                break;
            case DRIVE: color = Qt::gray;
                break;
            case IDLE: color = Qt::magenta;
                break;
        }
        return color;
    }
}
