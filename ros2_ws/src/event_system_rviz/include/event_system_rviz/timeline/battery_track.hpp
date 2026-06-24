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

    void updateScene(QGraphicsScene* scene, double pixelsPerSecond, int simStartTime, double xOffset, double yBase, int visStart, int visEnd) override {
        if (m_states.empty()) return;
        TimelineTransformer tf { pixelsPerSecond, simStartTime, xOffset };

        // draw soc range: x,y,w,h
        double xStart = tf.toX(m_states.front().time);
        double xEnd   = tf.toX(m_states.back().time);
        double width  = xEnd - xStart;
        double yBottom = yBase + m_height;

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

        // draw charge / discharge line: only the visible range plus the two
        // edge segments, dots thinned per pixel so item count tracks the viewport.
        double xPrev = 0.0, yPrev = 0.0;
        bool havePrev = false;
        double lastDotX = -1e9;
        for (size_t i = 0; i < m_states.size(); ++i) {
            const auto& state = m_states[i];
            const double x = tf.toX(state.time);
            const double y = yBottom - (m_height * state.props.soc);

            if (state.time < visStart) { xPrev = x; yPrev = y; havePrev = true; continue; }
            if (state.time > visEnd) {
                if (havePrev) { scene->addLine(xPrev, yPrev, x, y, { Qt::black, 1 }); }
                break;
            }

            if (havePrev) { scene->addLine(xPrev, yPrev, x, y, { Qt::black, 1 }); }

            if (x - lastDotX >= 2.0) {
                auto point = scene->addEllipse(
                    x - DOT_SIZE / 2.0,
                    y - DOT_SIZE / 2.0,
                    DOT_SIZE,
                    DOT_SIZE,
                    QPen(Qt::NoPen),
                    QBrush(Qt::black)
                );
                QString socStr = QString::fromStdString(std::to_string(state.props.soc));
                QString capacityStr = QString::fromStdString(std::to_string(state.props.capacity));
                point->setToolTip("SOC: " + socStr + " | " + capacityStr + " Ah - " + QString::fromStdString(des::toHumanReadableTime(state.time, true)));
                lastDotX = x;
            }

            xPrev = x;
            yPrev = y;
            havePrev = true;
        }
    }

    double getHeight() const override {
        return m_height;
    }

    void clear() override {
        m_states.clear();
    }
};
