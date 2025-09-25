#pragma once

#include <qgraphicsitem.h>
#include <QGraphicsView>

#include "Task.h"


// draw ordering
constexpr int Z_TIMELINE = 100;
constexpr int Z_TASKS = 50;

// constants
constexpr int LABEL_OFFSET = 20;
constexpr int X_LINE_POS = 0;
constexpr int Y_LINE_POS = 250;
constexpr int TASK_HEIGHT = 30;


class Timeline final : public QGraphicsView {
    Q_OBJECT
    QGraphicsScene* const m_scene;
    const int m_yLinePos = 250;
    const int m_endLinePos = 3600 * 12;
    const int m_tickStep = 10;

public:
    explicit Timeline(
        QGraphicsScene *scene,
        const int endLinePos,
        const int tickStep = 10,
        QWidget *parent=nullptr):
    QGraphicsView(scene, parent),
    m_scene(scene),
    m_endLinePos(endLinePos),
    m_tickStep(tickStep)
    {}

    void draw() const {
        // baseline of timeline (x-axis)
        QGraphicsLineItem* const line  = m_scene->addLine(
            X_LINE_POS,
            m_yLinePos,
            m_endLinePos,
            m_yLinePos,
            {Qt::black, 2}
            );

        line->setZValue(Z_TIMELINE);
        drawTicks();
    }

    void drawTask(const Task &task) const {
        const int start = X_LINE_POS + task.startTime;
        auto color = taskColor(task.type);
        QGraphicsRectItem* const rect = m_scene->addRect(
            start,
            m_yLinePos,
            task.duration,
            -TASK_HEIGHT,
            color,
            {color, Qt::SolidPattern}
            );
        rect->setZValue(Z_TASKS);
    }

    void drawTasks(const std::vector<Task> &tasks) const {
        for (auto const task : tasks) {
            drawTask(task);
        }
    }

private:
    void drawTicks() const {
        int time = 0;
        for (int xPosition = X_LINE_POS; xPosition <= m_endLinePos ;xPosition += m_tickStep) {
            if (time % 60 == 0) {
                constexpr int tickHeight = 5;
                QGraphicsLineItem* const line = m_scene->addLine(
                    xPosition,
                    m_yLinePos + tickHeight,
                    xPosition,
                    m_yLinePos - tickHeight,
                    {Qt::black, 2}
                    );
                line->setZValue(Z_TIMELINE);
                QGraphicsSimpleTextItem* const tickLabel = m_scene->addSimpleText(QString::number(time / 60));
                const double w = tickLabel->boundingRect().width();
                tickLabel->setPos(xPosition - w / 2.0, m_yLinePos + LABEL_OFFSET);
            } else {
                constexpr int smallTickHeight = 3;
                QGraphicsLineItem* const line =  m_scene->addLine(
                    xPosition,
                    m_yLinePos + smallTickHeight,
                    xPosition,
                    m_yLinePos - smallTickHeight,
                    {Qt::black, 1}
                    );
                line->setZValue(Z_TIMELINE);
            }
            time += m_tickStep;
        }
    }

};