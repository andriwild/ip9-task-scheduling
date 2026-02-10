#pragma once

#include <qnamespace.h>
#include <vector>
#include <QFontMetrics>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QPen>

#include "event_system_rviz/timeline/timeline_types.hpp"
#include "event_system_rviz/timeline/timeline_track.hpp"

class DriveTrack : public ITimelineTrack {
    int m_height;
    std::vector<std::pair<int,bool>> m_states;

public:
    DriveTrack(int height = 20) : m_height(height) {}

    std::string getName() const override { return "Drive"; }

    void handleStateChange(int time, bool isDriving) {
        m_states.push_back({time, isDriving});
    }

    void updateScene(QGraphicsScene* scene, double pixelsPerSecond, int simStartTime, double xOffset, double yBase) override {
        if (m_states.empty()) return;
        TimelineTransformer tf { pixelsPerSecond, simStartTime, xOffset };

        double xStart = tf.toX(m_states.front().first);
        double xEnd   = tf.toX(m_states.back().first);
        double width  = xEnd - xStart;
        QColor softGray(200, 200, 200, 40);

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
                double nextTime = (nextIt != m_states.end()) ? nextIt->first : time + 1.0;
        
                double xStart = tf.toX(time);
                double xEnd   = tf.toX(nextTime);
                double width  = xEnd - xStart;
        
                scene->addRect(
                    xStart,
                    yBase, 
                    width,
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
