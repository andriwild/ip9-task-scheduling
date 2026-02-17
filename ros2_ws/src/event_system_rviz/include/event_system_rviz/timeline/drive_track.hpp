#pragma once

#include <qnamespace.h>
#include <vector>
#include <QPen>

#include "event_system_rviz/timeline/timeline_types.hpp"
#include "event_system_rviz/timeline/timeline_track.hpp"

class DriveTrack final : public ITimelineTrack {
    int m_height;
    std::vector<std::pair<int,bool>> m_states;

public:
    explicit DriveTrack(const int height = 20) : m_height(height) {}

    std::string getName() const override { return "Drive"; }

    void handleStateChange(int time, bool isDriving) {
        m_states.emplace_back(time, isDriving);
    }

    void updateScene(QGraphicsScene* scene, const double pixelsPerSecond, const int simStartTime, const double xOffset, const double yBase) override {
        if (m_states.empty()) { return; }
        const TimelineTransformer tf { pixelsPerSecond, simStartTime, xOffset };

        const double xStart = tf.toX(m_states.front().first);
        const double xEnd   = tf.toX(m_states.back().first);
        const double width  = xEnd - xStart;
        constexpr QColor softGray(200, 200, 200, 40);

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
            auto [time, isDriving] = *it;
        
            if (isDriving) {
                auto nextIt = std::next(it);
                const double nextTime = (nextIt != m_states.end()) ? nextIt->first : time + 1.0;
        
                const double xStartPixel = tf.toX(time);
                const double xEndPixel   = tf.toX(nextTime);
                const double widthPixel  = xEndPixel - xStartPixel;
        
                scene->addRect(
                    xStartPixel,
                    yBase, 
                    widthPixel,
                    m_height,
                    QPen(Qt::NoPen),
                    QBrush(QColor(80, 200, 120))
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
