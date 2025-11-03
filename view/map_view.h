#pragma once

#include <QGraphicsView>
#include <QWheelEvent>
#include <iostream>
#include <QGraphicsLineItem>
#include <cmath>

#include "robot_item.h"
#include "../datastructure/graph.h"
#include "../model/simulation.h"

enum DIRECTION { N, O, S, W };

constexpr int Z_DRIVE = 1;
constexpr int Z_LOCATION = 100;
constexpr int Z_DOCK = Z_LOCATION - 1;
constexpr int Z_ROBOT= 200;

const std::string FILEPATH = "/home/andri/repos/ip9-task-scheduling/resources/";
const std::string FILENAME = "imvs2.pgm";

class MapView final : public QGraphicsView {
    Q_OBJECT
    QGraphicsScene *m_scene;
    bool m_userZoomed = false;
    Simulation& m_model;
    RobotItem *m_robotItem = new RobotItem();
    QGraphicsSimpleTextItem* m_timeCounter = nullptr;
    QGraphicsSimpleTextItem* m_stateLabel = nullptr;

public:
    explicit MapView(Simulation& model) :
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


        connect(&m_model, &Simulation::graphChanged,  this, &MapView::drawGraph);
        connect(&m_model, &Simulation::robotChanged,  this, &MapView::drawRobot);
        connect(&m_model, &Simulation::eventsChanged, this, &MapView::drawEvents);
        connect(&m_model, &Simulation::personChanged, this, &MapView::drawPersons);
        connect(&m_model, &Simulation::timeChanged,   this, &MapView::drawTimeLabel);

        drawGraph(m_model.getGraph());
        drawRobot(m_model.getRobot());
        drawEvents(m_model.getEvents());
        drawPersons(m_model.getPersonData());
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
        constexpr double radius = 5;
        constexpr auto color = Qt::red;
        m_scene->addEllipse(
            {pos.x() - radius / 2, pos.y() - radius / 2, radius, radius},
            {color},
            {color, Qt::SolidPattern}
        );

        QGraphicsView::mousePressEvent(event);
    }

    void keyPressEvent(QKeyEvent *event) override {
        if (event->key() == Qt::Key_Space) {
            m_model.simStep();
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

    void drawTimeLabel(const int time) {
        if (m_timeCounter == nullptr) {
            m_timeCounter = new QGraphicsSimpleTextItem();
            m_scene->addItem(m_timeCounter);
            QFont font = m_timeCounter->font();
            font.setBold(false);
            font.setPointSize(12);
            m_timeCounter->setFont(font);
            m_timeCounter->setBrush(Qt::black);
            m_timeCounter->setPos(width() - 40,20);
        }
        m_timeCounter->setText(QString::fromStdString(std::to_string(time)));
    }

    void drawStateLabel(Robot* robot) {
        if (m_stateLabel == nullptr) {
            m_stateLabel = new QGraphicsSimpleTextItem();
            m_scene->addItem(m_stateLabel);
            QFont font = m_stateLabel->font();
            font.setBold(false);
            font.setPointSize(12);
            m_stateLabel->setFont(font);
            m_stateLabel->setPos(width() - 140,20);
        }
        m_stateLabel->setBrush(Helper::color(robot->getTask()));
        m_stateLabel->setText(QString::fromStdString(Helper::toString(robot->getTask())));
    }

    void drawLocation(
        const Node &n,
        const std::string &label,
        const QColor &color = Qt::red,
        const DIRECTION labelPos = N,
        double radius = 10,
        int zValue = Z_LOCATION
    ) const {
        const auto point = m_scene->addEllipse(
            {n.x - radius / 2, n.y - radius / 2, radius, radius},
            {color},
            {color, Qt::SolidPattern}
        );
        point->setZValue(zValue);

        QGraphicsSimpleTextItem *const eventLabel = m_scene->addSimpleText(QString::fromStdString(label));
        const double w = eventLabel->boundingRect().width();
        switch (labelPos) {
            case N:  eventLabel->setPos(n.x - w, n.y - 2 * radius);
                break;
            case S: eventLabel->setPos(n.x - w, n.y + 1 * radius);
                break;
            case W: eventLabel->setPos(n.x - w  - 2 * radius, n.y);
                break;
            case O: eventLabel->setPos(n.x - w  + 2 * radius, n.y);
                break;
        }
        eventLabel->setZValue(Z_LOCATION + 1);
    }

    void drawPath(const Node &from, const Node &to, const QColor &color = Qt::black) const {
        const auto line = m_scene->addLine({from.x, from.y, to.x, to.y}, {color});
        line->setZValue(Z_DRIVE);
    }

    void drawGraph(Graph &graph) {
        auto nodes = graph.allNodes();
        for (auto [id, node]: graph.allNodes()) {
            drawLocation(node, std::to_string(id), Qt::darkGray);
        }
        for (auto [id, neighbours]: graph.allEdges()) {
            for (auto n: neighbours) {
                drawPath(nodes.at(id), nodes.at(n), Qt::lightGray);
            }
        }
    }

    void drawEvents(EV::Tree<SimulationEvent> &events) {
        auto evs = events.traversalPreOrder();
        for (auto ev: evs) {
            if (auto* escortEvent = dynamic_cast<MeetingEvent*>(ev->getValue())) {
               Node node =  m_model.getGraph().getNode(escortEvent->getDestination());
                drawLocation(node, std::to_string(escortEvent->getPersonId()), Helper::color(ROBOT_STATE::MEETING), S);
            }
        }
    }

    void drawRobot(Robot* robot) {
        const Graph graph = m_model.getGraph();
        const Node dock = graph.getNode(robot->getDock());
        drawLocation(dock, "Dock", Qt::yellow, O, 15, Z_DOCK);

        const Node pos = graph.getNode(robot->getPosition());
        if (robot->isDriving()) {
            const int targetId = robot->getTarget();
            const Node target = graph.getNode(targetId);
            m_robotItem->setDirection(target.x, target.y);
        } else {
            m_robotItem->clearDirection();
        }
        m_robotItem->setPos(pos.x - (15.0 / 2), pos.y - (15.0 / 2));

        drawStateLabel(robot);
    }

    void drawPersons(util::PersonData personData) {
        Graph graph = m_model.getGraph();
        for (auto p: personData) {
            int personId = p.first;
            auto searchLocation = p.second;
            for (auto sl: searchLocation) {
                drawLocation(graph.getNode(sl), std::to_string(personId), Helper::color(ROBOT_STATE::ESCORT), O);
            }
        }

    }
};
