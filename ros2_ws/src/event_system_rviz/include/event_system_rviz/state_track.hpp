#pragma once

#include "timeline_track.hpp"
#include "timeline_types.hpp"
#include <vector>
#include <QFontMetrics>

class StateTrack : public TimelineTrack {
    std::vector<VisualStateBlock> m_states;
    VisualStateBlock m_currentOpenState;
    int m_height;

public:
    StateTrack(int height = 20) : m_height(height) {
        m_currentOpenState = {-1, -1, 0};
    }

    void handleStateChange(int time, int newState, std::function<void()> updateCallback) {
        // Only close the previous state if it was valid (startTime != -1)
        if (m_currentOpenState.startTime != -1) {
            m_currentOpenState.endTime = time;
            m_states.push_back(m_currentOpenState);
        }

        m_currentOpenState.startTime = time;
        m_currentOpenState.endTime = -1;
        m_currentOpenState.type = newState;
        
        if(updateCallback) updateCallback();
    }

    void draw(QPainter* painter, const QRectF& rect, double pixelsPerSecond, int simStartTime, double xOffset, double yBase) override {

        // TODO: duplicate code
        auto timeToX = [&](int time) {
            return xOffset + (time - simStartTime) * pixelsPerSecond;
        };

        QFont font = painter->font();
        QFontMetrics fm(font);

        int barHeight = m_height;

        double viewLeft = rect.left();
        double viewRight = rect.right();

        for (const auto& block : m_states) {
            double x1 = timeToX(block.startTime);
            double x2 = timeToX(block.endTime);

            // Check visibility
            if (x2 < viewLeft || x1 > viewRight) {
                continue;
            }

            double blockLength = x2 - x1;
            QColor blockColor = getMeta(block.type).color;
            // Draw relative to yBase
            auto re = QRectF(x1, yBase, blockLength, barHeight); 
            painter->fillRect(re, blockColor);
            painter->setPen(QPen(blockColor.darker(300), 1));
            QString elidedLabel = fm.elidedText(QString::fromStdString(getMeta(block.type).label), Qt::TextElideMode::ElideRight, re.width());
            painter->drawText(re, Qt::AlignCenter, elidedLabel);
        }
    }

    double getHeight() const override {
        return m_height;
    }

    void clear() override {
        m_states.clear();
        m_currentOpenState = {-1, -1, 0};
    }
};
