#pragma once

#include <QGraphicsView>
#include <QWheelEvent>
#include <iostream>
#include <QGraphicsLineItem>
#include <qt5/QtWidgets/QGraphicsView>

#include "Event.h"

constexpr int MAP_HEIGHT = 400;
constexpr int MAP_WIDTH = 400;
constexpr int MAP_SCENE_MARGIN = 50;

constexpr int Z_DRIVE = 1;
constexpr int Z_LOCATION = 100;


class MapView final : public QGraphicsView {
    Q_OBJECT
    QGraphicsScene* m_scene;

public:
    explicit MapView() {
        m_scene = new QGraphicsScene(this);
        m_scene->setSceneRect(-MAP_SCENE_MARGIN, -MAP_SCENE_MARGIN, MAP_WIDTH, MAP_HEIGHT);
        setDragMode(ScrollHandDrag);
        setRenderHint(QPainter::Antialiasing);
        centerOn(0,0);
        setScene(m_scene);
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

    void driveFromTo(const Point &from, const Point &to, QColor color = Qt::black) const {
        const auto line = m_scene->addLine({ from.x, from.y, to.x, to.y }, { color });
        line->setZValue(Z_DRIVE);
    }

};
