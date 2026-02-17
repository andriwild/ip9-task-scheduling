#pragma once

#include <qnamespace.h>
#include <vector>
#include <QPen>

#include "event_system_rviz/timeline/timeline_types.hpp"
#include "event_system_rviz/timeline/timeline_track.hpp"

struct State {
    int time;
    bool isDriving;
    bool isCharging;
};

class DriveTrack final : public ITimelineTrack {
    int m_height;
    std::vector<State> m_states;
    const QColor driveColor  = QColor(80, 200, 120);
    const QColor chargeColor = QColor(241, 235, 156);

public:
    explicit DriveTrack(const int height = 20) : m_height(height) {}

    std::string getName() const override { return "Drive / Charge"; }

    // TODO: refactor to be extendable
    void handleStateChange(const int time, const bool isDriving, const bool isCharging) {
        m_states.push_back({ time, isDriving, isCharging });
    }

    void updateScene(QGraphicsScene* scene, const double pixelsPerSecond, const int simStartTime, const double xOffset, const double yBase) override {
        if (m_states.empty()) { return; }
        const TimelineTransformer tf { pixelsPerSecond, simStartTime, xOffset };

        const double xStart = tf.toX(m_states.front().time);
        const double xEnd   = tf.toX(m_states.back().time);
        const double width  = xEnd - xStart;
        constexpr QColor softGray(200, 200, 200, 40);

        // default state color
        scene->addRect(
            xStart,
            yBase,
            width,
            m_height,
            QPen(Qt::NoPen),
            QBrush(softGray)
        );

        // draw soc range: x,y,w,h
        for (auto it = m_states.begin(); it != m_states.end(); ++it) {
            const auto [time, isDriving, isCharging] = *it;
        
            if (isDriving || isCharging) {
                const auto color = isDriving ? driveColor : chargeColor;
                auto nextIt = std::next(it);
                const double nextTime = nextIt != m_states.end() ? nextIt->time: time + 1.0;
        
                const double xStartPixel = tf.toX(time);
                const double xEndPixel   = tf.toX(nextTime);
                const double widthPixel  = xEndPixel - xStartPixel;
        
                scene->addRect(
                    xStartPixel,
                    yBase, 
                    widthPixel,
                    m_height,
                    QPen(Qt::NoPen),
                    QBrush(color)
                );
            }
        }
    }

    double getHeight() const override {
        return m_height;
    }

    void clear() override {
        m_states.clear();
    }
};
