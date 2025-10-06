#pragma once

#include <qgraphicsitem.h>
#include <QGraphicsView>

#include "Event.h"
#include "Task.h"

constexpr int TIMELINE_HEIGHT = 500;
constexpr int TIMELINE_SCENE_MARGIN = 50;

// draw ordering
constexpr int Z_TIMELINE = 100;
constexpr int Z_TASKS = 50;
constexpr int Z_EVENT = 60;

// constants
constexpr int LABEL_OFFSET = 20;
constexpr int X_LINE_POS = TIMELINE_SCENE_MARGIN;
constexpr int Y_LINE_POS = 250;
constexpr int TASK_HEIGHT = 30;


class Timeline final : public QGraphicsView {
    Q_OBJECT
    QGraphicsScene* m_scene;
    const int m_yLinePos = 250;
    const int m_tickStep = 10;
    const int m_endLinePos;

public:
    explicit Timeline(const int simTime, const int tickStep = 10, QWidget *parent=nullptr):
    QGraphicsView(parent),
    m_tickStep(tickStep),
    m_endLinePos(simTime)
    {
        m_scene = new QGraphicsScene(this);
        m_scene->setSceneRect(-TIMELINE_SCENE_MARGIN, 0, simTime + TIMELINE_SCENE_MARGIN, TIMELINE_HEIGHT);
        setDragMode(ScrollHandDrag);
        setRenderHint(QPainter::Antialiasing);
        setScene(m_scene);
        centerOn(0,0);
    }

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

    void draw() const {
        // baseline of timeline (x-axis)
        QGraphicsLineItem* const line  = m_scene->addLine(
            X_LINE_POS,
            m_yLinePos,
            m_endLinePos - m_tickStep,
            m_yLinePos,
            {Qt::black, 2}
            );

        line->setZValue(Z_TIMELINE);
        drawTicks();
    }

    void drawEvent(const Event &event, const QColor &color = Qt::red) const {
        const int start = X_LINE_POS + event.time;
        constexpr int markerHeight = 30;
        QVector<QPoint> const points = {
            { start, m_yLinePos },
            { start + 10, m_yLinePos - 30 },
            { start - 10, m_yLinePos - 30 }
        };
        QPolygon const poly = points;
        const auto elem = m_scene->addPolygon(
            poly,
            { color },
            { color, Qt::SolidPattern }
            );
        elem->setZValue(Z_EVENT);

        QGraphicsSimpleTextItem* const eventLabel = m_scene->addSimpleText(QString::number(event.eventId));
        const double w = eventLabel->boundingRect().width();
        QFont font = eventLabel->font();
        font.setBold(true);
        font.setPointSize(8);
        eventLabel->setFont(font);
        eventLabel->setBrush(Qt::white);
        eventLabel->setPos(start - w / 2.0, m_yLinePos - markerHeight);
        eventLabel->setZValue(Z_EVENT + 1);
    }

    void drawTask(const Task &task) const {
        const int start = X_LINE_POS + task.startTime;
        const auto color = taskColor(task.type);

        QLinearGradient gradient(0, 0, 1 , 0);
        gradient.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient.setColorAt(0.0, color);
        gradient.setColorAt(1.0, Qt::white);

        QGraphicsRectItem* const rect = m_scene->addRect(
            start,
            m_yLinePos,
            task.duration,
            -TASK_HEIGHT,
            QPen(Qt::NoPen),
            QBrush(gradient)
            );
        rect->setZValue(Z_TASKS);
    }


    void drawDrive(const int startTime, const int endTime) const {
        const int start = X_LINE_POS + startTime;
        const auto color = taskColor(TaskType::DRIVE);

        QLinearGradient gradient(0, 0, 1 , 0);
        gradient.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient.setColorAt(0.0, color);
        gradient.setColorAt(1.0, Qt::white);

        QGraphicsRectItem* const rect = m_scene->addRect(
            start,
            m_yLinePos,
            endTime-startTime,
            -TASK_HEIGHT,
            QPen(Qt::NoPen),
            QBrush(gradient)
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
        for (int xPosition = X_LINE_POS; xPosition < m_endLinePos; xPosition += m_tickStep) {
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