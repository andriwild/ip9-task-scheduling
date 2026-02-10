#pragma once

#include <qnamespace.h>
#include <string>
#include <vector>
#include <QFontMetrics>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QPen>

#include "../../event_system_core/src/util/types.h"
#include "timeline_types.hpp"
#include "timeline_track.hpp"

struct VisualBatteryState {
    int time;
    des::BatteryProps props;
};

class BatteryTrack : public ITimelineTrack {
    std::vector<VisualBatteryState> m_states;
    int m_height;

public:
    BatteryTrack(int height = 20) : m_height(height) {}

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
        
        QColor softThreasholdRed(255, 0, 0, 20);
        QColor softRed(255, 0, 0, 60);      
        QColor softOrange(255, 165, 0, 50);
        QColor softGreen(0, 255, 0, 40);
        QColor softGray(200, 200, 200, 40);

        gradient.setColorAt(0.0, softRed);           
        gradient.setColorAt(threshold, softRed);      
        
        double orangePos = threshold + (1.0 - threshold) * 0.4; 
        gradient.setColorAt(orangePos, softOrange);
        
        gradient.setColorAt(1.0, softGreen);          
        
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
            yBottom - (m_height * threshold),
            width,
            (m_height * threshold),
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
            auto line = scene->addLine(x, yBottom, x, yBottom - m_height * state.props.soc, {Qt::gray, 2});
            QString socStr = QString::fromStdString(std::to_string(state.props.soc));
            QString capacityStr = QString::fromStdString(std::to_string(state.props.capacity));
            line->setToolTip("SOC: " + socStr + " | " + capacityStr +" Ah - " + QString::fromStdString(des::toHumanReadableTime(state.time, true)));

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
