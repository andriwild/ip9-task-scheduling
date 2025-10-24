#pragma once

#include <qgraphicsitem.h>
#include <QGraphicsView>

#include "event_plan.h"
#include "../datastructure/tree.h"
#include "../model/event.h"

constexpr int TIMELINE_HEIGHT = 700;
constexpr int TIMELINE_SCENE_MARGIN = 50;

// draw ordering
constexpr int Z_TIMELINE = 100;
constexpr int Z_TASKS = 50;
constexpr int Z_EVENT = 60;

// constants
constexpr int LABEL_OFFSET = 20;
constexpr int X_LINE_POS = TIMELINE_SCENE_MARGIN;
constexpr int Y_LINE_POS = 600;
constexpr int TASK_HEIGHT = 30;
constexpr double TIME_POINTER_HEIGHT = 200.0;



class Timeline final : public QGraphicsView {
    Q_OBJECT
    QGraphicsScene* m_scene;
    Simulation& m_model;
    const int m_yLinePos = Y_LINE_POS;
    const int m_tickStep = 10;
    const int m_endLinePos;
    QGraphicsLineItem* m_timePointer;

public:
    explicit Timeline(Simulation& model, const int simTime, const int tickStep = 10, QWidget *parent=nullptr):
    QGraphicsView(parent),
    m_model(model),
    m_tickStep(tickStep),
    m_endLinePos(simTime)
    {
        m_scene = new QGraphicsScene(this);
        m_scene->setSceneRect(-TIMELINE_SCENE_MARGIN, 0, simTime + TIMELINE_SCENE_MARGIN, TIMELINE_HEIGHT);
        setDragMode(ScrollHandDrag);
        setRenderHint(QPainter::Antialiasing);
        setScene(m_scene);
        centerOn(0,0);
        m_timePointer = m_scene->addLine( X_LINE_POS, m_yLinePos - TIME_POINTER_HEIGHT / 2, X_LINE_POS, m_yLinePos , {Qt::red} );

        connect(&m_model, &Simulation::timeChanged, this, &Timeline::drawPointer);
        connect(&m_model, &Simulation::eventsChanged, this, &Timeline::drawEvents);

        drawPointer(0);
        drawBaseline();
        drawEvents(m_model.getEvents());
    }

    void drawPointer(const int time) {
        m_timePointer->setLine( X_LINE_POS + time, m_yLinePos - Y_LINE_POS + TIMELINE_SCENE_MARGIN, X_LINE_POS + time, m_yLinePos );
        update();
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

        // there is no easy way to get all enum values
        std::array<std::pair<QColor, std::string>, 7> allTaskTypes = {
            {
                {Helper::color(IDLE),    Helper::typeToString(IDLE)},
                {Helper::color(DRIVE),   Helper::typeToString(DRIVE)},
                {Helper::color(ESCORT),  Helper::typeToString(ESCORT)},
                {Helper::color(SEARCH),  Helper::typeToString(SEARCH)},
                {Helper::color(MEETING), Helper::typeToString(MEETING)},
                {Helper::color(TOUR),    Helper::typeToString(TOUR)},
                {Helper::color(TALK),    Helper::typeToString(TALK)},
            }
        };        int rectPos = 10;
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

    void drawBaseline() const {
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

    void drawEvents(Tree<SimulationEvent> &events) const {
        assert(events.getRoot()->parent == nullptr);
        std::vector<EventPlan*> plans;
        for (const auto& child: events.getRoot()->getChildren()) {
            auto eventPlan = new EventPlan(*child.get(), X_LINE_POS);
            plans.push_back(eventPlan);
        }
        std::ranges::reverse(plans);

        int offset = Y_LINE_POS - 25;
        for (const auto plan: plans) {
            m_scene->addItem(plan);
            offset -= plan->boundingRect().height() + 25;
            plan->setPos(0, offset);
        }
    }

    void drawBlock(const int startTime, const int endTime, const QColor &color = Qt::gray) const {
        const int duration = endTime - startTime;
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
            if (time % 100 == 0) {
                constexpr int tickHeight = 5;
                QGraphicsLineItem* const line = m_scene->addLine(
                    xPosition,
                    m_yLinePos + tickHeight,
                    xPosition,
                    m_yLinePos - tickHeight,
                    {Qt::black, 2}
                    );
                line->setZValue(Z_TIMELINE);
                QGraphicsSimpleTextItem* const tickLabel = m_scene->addSimpleText(QString::number(time));
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