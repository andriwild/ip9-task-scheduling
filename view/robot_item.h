#pragma once
#include <qgraphicsitem.h>
#include <QGraphicsView>

constexpr int ROBOT_SIZE = 50;
constexpr double WHEEL_WIDTH = 15.0;

class RobotItem : public QGraphicsItemGroup {
    QGraphicsLineItem* m_directionLine = nullptr;

public:
    explicit RobotItem(QGraphicsItem* parent = nullptr)
        : QGraphicsItemGroup(parent) {

        auto body = new QGraphicsRectItem(0, 0, ROBOT_SIZE, ROBOT_SIZE, this);
        body->setPen(QPen(Qt::black, 1));
        body->setBrush(QColor(200, 150, 0));

        createWheel(-WHEEL_WIDTH / 2, WHEEL_WIDTH / 4); // top left
        createWheel(ROBOT_SIZE, WHEEL_WIDTH / 4); // top right
        createWheel(-WHEEL_WIDTH / 2, ROBOT_SIZE - WHEEL_WIDTH - WHEEL_WIDTH / 4); // bottom left
        createWheel(ROBOT_SIZE, ROBOT_SIZE - WHEEL_WIDTH - WHEEL_WIDTH / 4); // bottom right
    }

    void setDirection(const double x, const double y) {
        clearDirection();

        QPointF center(ROBOT_SIZE / 2, ROBOT_SIZE / 2);
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
        auto wheel = new QGraphicsRectItem(x, y, WHEEL_WIDTH / 2, WHEEL_WIDTH, this);
        wheel->setPen(QPen(Qt::black, 1));
        wheel->setBrush(QColor(40, 40, 40));
    }
};