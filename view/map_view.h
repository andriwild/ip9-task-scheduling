#pragma once

#include <QGraphicsView>
#include <QWheelEvent>
#include <iostream>
#include <QGraphicsLineItem>
#include <qt5/QtWidgets/QGraphicsView>
#include <cmath>

#include "robot_item.h"
#include "../model/event.h"
#include "../datastructure/graph.h"
#include "../model/simulation_model.h"

constexpr int Z_DRIVE = 1;
constexpr int Z_LOCATION = 100;
constexpr int Z_ROBOT= 200;

const std::string FILEPATH = "/home/andri/repos/ip9-task-scheduling/resources/";
const std::string FILENAME = "imvs2.pgm";

class MapView final : public QGraphicsView {
    Q_OBJECT

    QGraphicsScene *m_scene;
    bool m_userZoomed = false;
    SimulationModel& m_model;
    RobotItem *m_robotItem = new RobotItem();

public:
    explicit MapView(SimulationModel& model) :
    m_model(model) {
        m_scene = new QGraphicsScene(this);
        setDragMode(ScrollHandDrag);
        setRenderHint(QPainter::Antialiasing);
        setTransformationAnchor(AnchorUnderMouse);
        setScene(m_scene);
        const QPixmap map((FILEPATH + FILENAME).data());
        m_scene->addPixmap(map);
        m_scene->addItem(m_robotItem);
        m_robotItem->setZValue(Z_ROBOT);
        connect(&m_model, &SimulationModel::graphChanged, this, &MapView::drawGraph);
        connect(&m_model, &SimulationModel::robotChanged, this, &MapView::drawRobot);
        drawGraph(m_model.getGraph());
        drawRobot(m_model.getRobot());
    }

    void showEvent(QShowEvent *event) override {
        QGraphicsView::showEvent(event);
        fitInView(sceneRect(), Qt::KeepAspectRatio);
    }

    void wheelEvent(QWheelEvent *event) override {
        m_userZoomed = true;
        const double factor = event->angleDelta().y() > 0 ? 1.15 : 0.85;
        scale(factor, factor);
    }

    void resizeEvent(QResizeEvent *event) override {
        QGraphicsView::resizeEvent(event);
        if (!m_userZoomed) {
            fitInView(sceneRect(), Qt::KeepAspectRatio);
        }
    }

    void mousePressEvent(QMouseEvent *event) override {
        const auto pos = mapToScene(event->pos());
        std::cout << pos.x() << "," << pos.y() << std::endl;
        const double radius = 5;
        const auto color = Qt::red;
        m_scene->addEllipse(
            {pos.x() - radius / 2, pos.y() - radius / 2, radius, radius},
            {color},
            {color, Qt::SolidPattern}
        );

        QGraphicsView::mousePressEvent(event);
    }

    void keyPressEvent(QKeyEvent *event) override {
        if (event->key() == Qt::Key_Space) {
            m_model.step();
        } else {
            QGraphicsView::keyPressEvent(event);
        }
    }

    double width() const {
        return m_scene->width();
    }

    double height() const {
        return m_scene->height();
    }

    void drawForeground(QPainter *painter, const QRectF &rect) override {
    }

    // void drawBackground(QPainter *painter, const QRectF &rect) override {
    //     QGraphicsView::drawBackground(painter, rect);
    //
    //     const double gridStep = 100.0;
    //
    //     painter->fillRect(rect, Qt::white);
    //
    //     QPen gridPen(QColor(200, 200, 200), 0.5);
    //     painter->setPen(gridPen);
    //
    //     double startX = std::floor(rect.left() / gridStep) * gridStep;
    //     for (double x = startX; x <= rect.right(); x += gridStep) {
    //         painter->drawLine(QNodeF(x, rect.top()), QNodeF(x, rect.bottom()));
    //     }
    //
    //     double startY = std::floor(rect.top() / gridStep) * gridStep;
    //     for (double y = startY; y <= rect.bottom(); y += gridStep) {
    //         painter->drawLine(QNodeF(rect.left(), y), QNodeF(rect.right(), y));
    //     }
    //
    //     QPen axisPen(QColor(150, 150, 150), 1.0);
    //     painter->setPen(axisPen);
    //     painter->drawLine(QNodeF(rect.left(), 0), QNodeF(rect.right(), 0));
    //     painter->drawLine(QNodeF(0, rect.top()), QNodeF(0, rect.bottom()));
    // }

    void drawLocation(const Node &n, const std::string &label, const QColor &color = Qt::red,
                      double radius = 10) const {
        const auto point = m_scene->addEllipse(
            {n.x - radius / 2, n.y - radius / 2, radius, radius},
            {color},
            {color, Qt::SolidPattern}
        );
        point->setZValue(Z_LOCATION);

        QGraphicsSimpleTextItem *const eventLabel = m_scene->addSimpleText(QString::fromStdString(label));
        const double w = eventLabel->boundingRect().width();
        eventLabel->setPos(n.x - w, n.y - 2 * radius);
        eventLabel->setZValue(Z_LOCATION + 1);
    }

    void drawPath(const Node &from, const Node &to, const QColor &color = Qt::black) const {
        const auto line = m_scene->addLine({from.x, from.y, to.x, to.y}, {color});
        line->setZValue(Z_DRIVE);
    }

    void drawGraph(Graph &graph) {
        auto nodes = graph.allNodes();
        for (auto [id, node]: graph.allNodes()) {
            drawLocation(node, std::to_string(id));
        }
        std::cout << graph << std::endl;
        for (auto [id, neighbours]: graph.allEdges()) {
            for (auto n: neighbours) {
                drawPath(nodes.at(id), nodes.at(n), Qt::lightGray);
            }
        }
    }

    void drawRobot(Robot& robot) {
        auto pos = robot.getPosition();
        if (robot.isDriving()) {
            auto target = robot.getTarget().value();
            m_robotItem->setDirection(target.x, target.y);
        } else {
            m_robotItem->clearDirection();
        }
        m_robotItem->setPos(pos.x - (15.0 / 2), pos.y - (15.0 / 2));
    }
};
