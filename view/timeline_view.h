#pragma once

#include <qgraphicsitem.h>
#include <QGraphicsView>

#include "../model/event.h"

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
            const double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
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
                {Qt::darkGreen, "Escort"},
                {Qt::gray, "Drive"},
                {Qt::blue, "Dock"},
                //{{255, 165, 0}, "Charge"},
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

    void drawEvent(
        const int time,
        const std::string &label,
        const QColor &color = Qt::red,
        const bool includeLabel = false
    ) const {
        const int start = X_LINE_POS + time;
        constexpr int markerHeight = 30;
        const int yMarkerPos = m_yLinePos - TASK_HEIGHT;
        QVector<QPoint> const points = {
            { start, yMarkerPos },
            { start + 10, yMarkerPos - 30 },
            { start - 10, yMarkerPos - 30 }
        };
        QPolygon const poly = points;
        const auto elem = m_scene->addPolygon(poly, { color }, { color, Qt::SolidPattern });
        elem->setZValue(Z_EVENT);

        QGraphicsSimpleTextItem* const eventLabel = m_scene->addSimpleText(QString::fromStdString(label));
        const double w = eventLabel->boundingRect().width();
        const double h = eventLabel->boundingRect().height();
        if (includeLabel) {
            QFont font = eventLabel->font();
            font.setBold(true);
            font.setPointSize(8);
            eventLabel->setFont(font);
            eventLabel->setBrush(Qt::white);
            eventLabel->setPos(start - w / 2.0, yMarkerPos - markerHeight);
        } else {
            eventLabel->setPos(start - w / 2.0, yMarkerPos - markerHeight - h);
        }
        eventLabel->setZValue(Z_EVENT + 1);
    }

    void drawDrive(const int startTime, const int duration, const QColor &color = Qt::gray) const {
        const int start = X_LINE_POS + startTime;

        QLinearGradient gradient(0, 0, 1 , 0);
        gradient.setCoordinateMode(QGradient::ObjectBoundingMode);
        gradient.setColorAt(0.0, color);
        gradient.setColorAt(1.0, Qt::white);

        QGraphicsRectItem* const rect = m_scene->addRect(
            start,
            m_yLinePos,
            duration,
            -TASK_HEIGHT,
            QPen(Qt::black),
            QBrush(gradient)
            );
        rect->setZValue(Z_TASKS);
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