#pragma once
#include <qgraphicsitem.h>
#include <QGraphicsView>


class RobotItem : public QGraphicsItemGroup {
    QGraphicsLineItem* m_directionLine = nullptr;

public:
    explicit RobotItem(QGraphicsItem* parent = nullptr)
        : QGraphicsItemGroup(parent) {

        auto body = new QGraphicsRectItem(0, 0, 15, 15, this);
        body->setPen(QPen(Qt::black, 1));
        body->setBrush(QColor(200, 150, 0));

        createWheel(-2, 2);
        createWheel(15, 2);
        createWheel(-2, 10);
        createWheel(15, 10);
    }

    void setDirection(const double x, const double y) {
        clearDirection();

        QPointF center(7.5, 7.5);
        QPointF target = mapFromScene(QPointF(x, y));

        m_directionLine = new QGraphicsLineItem(
            center.x(), center.y(),
            target.x(), target.y(),
            this
        );

        QPen linePen(QColor(255, 0, 0, 150), 2);
        linePen.setStyle(Qt::DashLine);
        m_directionLine->setPen(linePen);
        m_directionLine->setZValue(-1);
    }

    void clearDirection() {
        if (m_directionLine) {
            delete m_directionLine;
            m_directionLine = nullptr;
        }
    }

private:
    void createWheel(qreal x, qreal y) {
        auto wheel = new QGraphicsRectItem(x, y, 2, 4, this);
        wheel->setPen(QPen(Qt::black, 1));
        wheel->setBrush(QColor(40, 40, 40));
    }
};