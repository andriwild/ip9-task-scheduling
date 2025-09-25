#pragma once
#include <QGraphicsView>
#include <QWheelEvent>
#include <iostream>

#include "Task.h"

class MainView final : public QGraphicsView {
    Q_OBJECT

public:
    void wheelEvent(QWheelEvent *event) override {
        if (event->modifiers() & Qt::ControlModifier) {
            const double factor = (event->angleDelta().y() > 0) ? 1.15 : 1.0 / 1.15;
            scale(factor, factor);
        } else {
            QGraphicsView::wheelEvent(event);
        }
    }

    void drawForeground(QPainter *painter, const QRectF &rect) override {
        painter->save();
        painter->resetTransform();

        // there is no easy way to get all enum values -> duplicated code
        std::array<std::pair<QColor, std::string>, 3> allTaskTypes = {
            {
                {taskColor(TaskType::ESCORT), "Escort"},
                {taskColor(TaskType::DRIVE), "Drive"},
                {taskColor(TaskType::CHARGE), "Charge"},
            }
        };
        int rectPos = 10;
        for (const auto& [color, label] : allTaskTypes) {
            constexpr int margin = 15;
            constexpr int labelDistance = 80;

            painter->setBrush(color);
            painter->setPen(color);
            painter->drawRect(rectPos, 10, 10, 10);
            painter->setPen(Qt::black);
            painter->drawText(rectPos + margin, 20, QString::fromStdString(label));
            rectPos += labelDistance;
        }
        painter->restore();
    }
};
