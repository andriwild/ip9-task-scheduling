#pragma once
#include <qcolor.h>

#include "../model/robot.h"

namespace Helper {
    inline QColor color(ROBOT_STATE task) {
        QColor color = Qt::black;
        switch (task) {
            case ROBOT_STATE::IDLE:
                color = QColor(236, 213, 188);
                break;
            case ROBOT_STATE::DRIVE:
                color = QColor(117, 138, 147);
                break;
            case ROBOT_STATE::ESCORT:
                color = QColor(233, 182, 59);
                break;
            case ROBOT_STATE::SEARCH:
                color = QColor(198, 110, 82);
                break;
            case ROBOT_STATE::MEETING:
                color = QColor(255, 228, 196);
                break;
            case ROBOT_STATE::TOUR:
                color = QColor(169, 192, 176);
                break;
            case ROBOT_STATE::TALK:
                color = QColor(180, 215, 230);
                break;
        }
        return color;
    }
    inline std::string colorAnsi(const ROBOT_STATE task) {
        std::string color = "\033[0m";
        switch (task) {
            case ROBOT_STATE::IDLE:
                color = "\033[38;2;236;213;188m";
                break;
            // case DRIVE:
            //     color = "\033[38;2;117;138;147m";
            //     break;
            case ROBOT_STATE::ESCORT:
                color = "\033[38;2;233;182;59m";
                break;
            case ROBOT_STATE::SEARCH:
                color = "\033[38;2;198;110;82m";
                break;
            case ROBOT_STATE::MEETING:
                color = "\033[38;2;255;228;196m";
                break;
            case ROBOT_STATE::TOUR:
                color = "\033[38;2;169;192;176m";
                break;
            case ROBOT_STATE::TALK:
                color = "\033[38;2;210;180;140m";
                break;
        }
        return color;
    }
    inline std::string toString(ROBOT_STATE task) {
        switch (task) {
            case ROBOT_STATE::IDLE:
                return "Idle";
            case ROBOT_STATE::DRIVE:
                return "Drive";
            case ROBOT_STATE::ESCORT:
                return "Escort";
            case ROBOT_STATE::SEARCH:
                return "Search";
            case ROBOT_STATE::MEETING:
                return "Meeting";
            case ROBOT_STATE::TOUR:
                return "Tour";
            case ROBOT_STATE::TALK:
                return "Talk";
            default:
                return "Unknown";
        }
    }
}