#pragma once

#include "event_system_rviz/timeline/timeline_track.hpp"
#include "event_system_rviz/timeline/timeline_types.hpp"
#include <vector>
#include <QFontMetrics>
#include <QGraphicsRectItem>
#include <QPen>

class StateTrack final : public ITimelineTrack {
    std::vector<VisualStateBlock> m_states;
    VisualStateBlock m_currentOpenState{};
    int m_height;

public:
    explicit StateTrack(const int height = 20) : m_height(height) {
        m_currentOpenState = {-1, -1, 0};
    }

    std::string getName() const override { return "State"; }

    void handleStateChange(const int time, const int newState) {
        // Only close the previous state if it was valid (startTime != -1)
        if (m_currentOpenState.startTime != -1) {
            m_currentOpenState.endTime = time;
            m_states.push_back(m_currentOpenState);
        }

        m_currentOpenState.startTime = time;
        m_currentOpenState.endTime = -1;
        m_currentOpenState.type = newState;
    }

    void updateScene(QGraphicsScene* scene, double pixelsPerSecond, int simStartTime, double xOffset, double yBase) override {
        const TimelineTransformer tf { pixelsPerSecond, simStartTime, xOffset };

        int barHeight = m_height;

        for (const auto&[startTime, endTime, type] : m_states) {
            double x1 = tf.toX(startTime);
            double x2 = tf.toX(endTime);
            double blockLength = x2 - x1;

            if (blockLength <= 0) continue;

            RobotStateMeta meta = getMeta(type);
            QColor blockColor = meta.color;
            QString labelStr = QString::fromStdString(meta.label);

            QRectF rect(x1, yBase, blockLength, barHeight);
            auto rectItem = scene->addRect(rect, QPen(Qt::NoPen), QBrush(blockColor));
            
            rectItem->setToolTip(labelStr + ": " + QString::fromStdString(des::toHumanReadableTime(startTime, true)) + " - " + QString::fromStdString(des::toHumanReadableTime(endTime, true)));
            
            QFont font;
            font.setPointSize(8);
            QFontMetrics fm(font);
            
            QString elidedText = fm.elidedText(labelStr, Qt::ElideRight, blockLength - 4); // -4 for padding
            
            if (!elidedText.isEmpty()) {
                auto textItem = scene->addText(elidedText);
                textItem->setFont(font);
                textItem->setDefaultTextColor(blockColor.darker(300));
                
                QRectF textRect = textItem->boundingRect();
                double textX = x1 + (blockLength - textRect.width()) / 2.0;
                double textY = yBase + (barHeight - textRect.height()) / 2.0;
                
                if (textX < x1) { textX = x1; }
                
                textItem->setPos(textX, textY);
                textItem->setZValue(rectItem->zValue() + 1);
            }
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
