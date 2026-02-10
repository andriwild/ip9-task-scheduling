#pragma once

#include <qnamespace.h>
#include <string>
#include <vector>
#include <QFontMetrics>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QPen>

#include "../../event_system_core/src/util/types.h"
#include "event_system_rviz/timeline/timeline_types.hpp"
#include "event_system_rviz/timeline/timeline_track.hpp"


constexpr double DOT_SIZE = 5.0;

struct VisualBatteryState {
    int time;
    des::BatteryProps props;
};

class BatteryTrack : public ITimelineTrack {
    std::vector<VisualBatteryState> m_states;
    int m_height;

public:
    BatteryTrack(int height = 20) : m_height(height) {}

    std::string getName() const override { return "Battery"; }

    void handleStateChange(int time, des::BatteryProps props) {
        m_states.push_back({time, props});
    }

    void updateScene(QGraphicsScene* scene, double pixelsPerSecond, int simStartTime, double xOffset, double yBase) override {
        if (m_states.empty()) return;
        TimelineTransformer tf { pixelsPerSecond, simStartTime, xOffset };

        // draw soc range: x,y,w,h
        double xStart = tf.toX(m_states.front().time);
        double xEnd   = tf.toX(m_states.back().time);
        double width  = xEnd - xStart;
        double yBottom = yBase + m_height;
        
        QLinearGradient gradient(xStart, yBase + m_height, xStart, yBase);
        
        double threshold = m_states.front().props.lowThreshold / 100.0;
        
        QColor softRed(255, 0, 0, 60);      
        QColor softGray(200, 200, 200, 40);
        
        // battery range
        scene->addRect(
            xStart,
            yBase,
            width,
            m_height,
            QPen(Qt::NoPen),
            QBrush(softGray)
        );
        
        // threshold range
        scene->addRect(
            xStart,
            yBottom,
            width,
            -(m_height * threshold),
            QPen(Qt::NoPen),
            QBrush(softRed)
        );

        // draw charge / discharge line
        double xPrev, yPrev;
        for (size_t i = 0; i < m_states.size(); ++i) {
            const auto& state = m_states[i];
            double x = tf.toX(state.time);

            if(i > 0){
                scene->addLine(
                    xPrev,
                    yPrev,
                    x,
                    yBottom - m_height * state.props.soc, 
                    { Qt::black, 1 }
                );
            }

            double yTop = yBottom - (m_height * state.props.soc);

            auto point = scene->addEllipse(
                x - DOT_SIZE/ 2.0,
                yTop - DOT_SIZE/ 2.0,
                DOT_SIZE, 
                DOT_SIZE, 
                QPen(Qt::NoPen),
                QBrush(Qt::black)
            );
            QString socStr = QString::fromStdString(std::to_string(state.props.soc));
            QString capacityStr = QString::fromStdString(std::to_string(state.props.capacity));
            point->setToolTip("SOC: " + socStr + " | " + capacityStr +" Ah - " + QString::fromStdString(des::toHumanReadableTime(state.time, true)));

            QFont font;
            font.setPointSize(8);
            QFontMetrics fm(font);
            xPrev = x;
            yPrev = yBottom - m_height * state.props.soc;
        }
    }

    double getHeight() const override {
        return m_height;
    }

    void clear() override {
        m_states.clear();
    }
};
