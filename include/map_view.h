#pragma once

#include <QGraphicsView>
#include <QWheelEvent>
#include <iostream>
#include <QGraphicsLineItem>
#include <qt5/QtWidgets/QGraphicsView>
#include <cmath>

#include "event.h"

constexpr int MAP_HEIGHT = 1000;
constexpr int MAP_WIDTH = 1000;
constexpr int MAP_SCENE_MARGIN = 50;

constexpr int Z_DRIVE = 1;
constexpr int Z_LOCATION = 100;


class map_view final : public QGraphicsView {
    Q_OBJECT
    QGraphicsScene* m_scene;

public:
    explicit map_view() {
        m_scene = new QGraphicsScene(this);
        m_scene->setSceneRect(-MAP_WIDTH / 2, -MAP_HEIGHT / 2, MAP_WIDTH, MAP_HEIGHT);
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

    void drawForeground(QPainter *painter, const QRectF &rect) override { }

    void drawBackground(QPainter *painter, const QRectF &rect) override {
        QGraphicsView::drawBackground(painter, rect);

        const double gridStep = 100.0;

        painter->fillRect(rect, Qt::white);

        QPen gridPen(QColor(200, 200, 200), 0.5);
        painter->setPen(gridPen);

        double startX = std::floor(rect.left() / gridStep) * gridStep;
        for (double x = startX; x <= rect.right(); x += gridStep) {
            painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
        }

        double startY = std::floor(rect.top() / gridStep) * gridStep;
        for (double y = startY; y <= rect.bottom(); y += gridStep) {
            painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
        }

        QPen axisPen(QColor(150, 150, 150), 1.0);
        painter->setPen(axisPen);
        painter->drawLine(QPointF(rect.left(), 0), QPointF(rect.right(), 0));
        painter->drawLine(QPointF(0, rect.top()), QPointF(0, rect.bottom()));
    }

    void drawLocation(const Point &p, const std::string &label, const QColor& color = Qt::red, double radius = 10) const {
        const auto point = m_scene->addEllipse(
            { p.x - radius / 2, p.y - radius / 2, radius, radius },
            { color },
            { color, Qt::SolidPattern }
            );
        point->setZValue(Z_LOCATION);

        QGraphicsSimpleTextItem* const eventLabel = m_scene->addSimpleText(QString::fromStdString(label));
        const double w = eventLabel->boundingRect().width();
        eventLabel->setPos(p.x - w, p.y - 2 * radius);
        eventLabel->setZValue(Z_LOCATION + 1);
    }

    void drawPath(const Point &from, const Point &to, const QColor& color = Qt::black) const {
        const auto line = m_scene->addLine({ from.x, from.y, to.x, to.y }, { color });
        line->setZValue(Z_DRIVE);
    }

};
