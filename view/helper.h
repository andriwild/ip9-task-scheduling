#pragma once
#include <qcolor.h>

#include "../model/robot.h"

namespace Helper {
    inline QColor color(TYPE task) {
        QColor color = Qt::black;
        switch (task) {
            case IDLE:
                color = QColor(236, 213, 188);
                break;
            case DRIVE:
                color = QColor(117, 138, 147);
                break;
            case ESCORT:
                color = QColor(233, 182, 59);
                break;
            case SEARCH:
                color = QColor(198, 110, 82);
                break;
            case MEETING:
                color = QColor(255, 228, 196);
                break;
            case TOUR:
                color = QColor(169, 192, 176);
                break;
            case TALK:
                color = QColor(180, 215, 230);
                break;
        }
        return color;
    }
    inline std::string colorAnsi(const TYPE task) {
        std::string color = "\033[0m";
        switch (task) {
            case IDLE:
                color = "\033[38;2;236;213;188m";
                break;
            // case DRIVE:
            //     color = "\033[38;2;117;138;147m";
            //     break;
            case ESCORT:
                color = "\033[38;2;233;182;59m";
                break;
            case SEARCH:
                color = "\033[38;2;198;110;82m";
                break;
            case MEETING:
                color = "\033[38;2;255;228;196m";
                break;
            case TOUR:
                color = "\033[38;2;169;192;176m";
                break;
            case TALK:
                color = "\033[38;2;210;180;140m";
                break;
        }
        return color;
    }
    inline std::string typeToString(TYPE task) {
        switch (task) {
            case IDLE:
                return "Idle";
            case DRIVE:
                return "Drive";
            case ESCORT:
                return "Escort";
            case SEARCH:
                return "Search";
            case MEETING:
                return "Meeting";
            case TOUR:
                return "Tour";
            case TALK:
                return "Talk";
            default:
                return "Unknown";
        }
    }
}