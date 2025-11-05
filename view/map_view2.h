#pragma once

#include <QGraphicsView>
#include <QWheelEvent>
#include <iostream>
#include <QGraphicsLineItem>
#include <cmath>

#include "map.h"
#include "robot_item.h"
#include "../datastructure/graph.h"
#include "../model/simulation.h"
#include "../util/vis_lib.h"

enum DIRECTION { N, O, S, W };

constexpr int Z_DRIVE = 1;
constexpr int Z_LOCATION = 100;
constexpr int Z_DOCK = Z_LOCATION - 1;
constexpr int Z_ROBOT = 200;

const std::string RES_PATH = "/home/andri/repos/ip9-task-scheduling/resources/";
const std::string MAP_FILENAME = "IMVS_data.bin";

class MapView final : public QGraphicsView {
    Q_OBJECT
    QGraphicsScene *m_scene;
    bool m_userZoomed = false;
    Simulation &m_model;
    RobotItem *m_robotItem = new RobotItem();
    QGraphicsSimpleTextItem *m_timeCounter = nullptr;
    QGraphicsSimpleTextItem *m_stateLabel = nullptr;
    std::vector<VisLib::Segment> segments;
    std::vector<VisLib::Point> points;

public:
    explicit MapView(Simulation &model) : m_model(model) {
        m_scene = new QGraphicsScene(this);
        setDragMode(ScrollHandDrag);
        setRenderHint(QPainter::Antialiasing);
        setTransformationAnchor(AnchorUnderMouse);
        setScene(m_scene);

        //m_scene->addItem(m_robotItem);
        //m_robotItem->setZValue(Z_ROBOT);


        connect(&m_model, &Simulation::graphChanged, this, &MapView::drawGraph);
        connect(&m_model, &Simulation::robotChanged, this, &MapView::drawRobot);
        connect(&m_model, &Simulation::eventsChanged, this, &MapView::drawEvents);
        connect(&m_model, &Simulation::personChanged, this, &MapView::drawPersons);
        connect(&m_model, &Simulation::timeChanged, this, &MapView::drawTimeLabel);

        // drawGraph(m_model.getGraph());
        // drawRobot(m_model.getRobot());
        // drawEvents(m_model.getEvents());
        // drawPersons(m_model.getPersonData());

        VisLib::readFile(RES_PATH + MAP_FILENAME, segments, points);

        const double scale = 100.0;
        for (auto& s: segments) {
            s.m_points[0].m_x *= scale;
            s.m_points[0].m_y *= scale;
            s.m_points[1].m_x *= scale;
            s.m_points[1].m_y *= scale;
        }

        auto m = new Map(segments, points);
        m_scene->addItem(m);
        m_scene->setSceneRect(m_scene->itemsBoundingRect());

        QRectF bounds = m->boundingRect();
        const int h = static_cast<int>(bounds.height());
        const int w = static_cast<int>(bounds.width());
        double offsetX = bounds.x();
        double offsetY = bounds.y();

        std::vector<std::vector<Node>> girdLines;
        Graph gridGraph;
        double gridSize = 50.0;
        createGridGraph(offsetX, offsetY, h, w, gridGraph, gridSize, girdLines);
        connectAllGridNodes(gridGraph, girdLines);
        removeIntersectingEdges(gridGraph, gridSize, girdLines, offsetX, offsetY);
        drawGraph(gridGraph);
    }

    void connectAllGridNodes(Graph& gridGraph, const std::vector<std::vector<Node>>& lines) {
        Log::d("[ MapView ] Connecting all Grid nodes by edges");
        std::vector<Node> prevLine;
        for (auto l : lines) {
            for (int j = 0; j < l.size(); ++j) {
                // horizontal
                if (j + 1 < l.size()) {
                    gridGraph.addEdge(l[j].id, l[j + 1].id);
                }
                // vertical
                if (!prevLine.empty()) {
                    gridGraph.addEdge(prevLine[j].id, l[j].id);
                }
            }
            prevLine = l;
        }
    }

    void createGridGraph(double &offsetX, double &offsetY, double h, double w, Graph &gridGraph, double &gridSize, std::vector<std::vector<Node>> &lines) {
        Log::d("[ MapView ] Create Grid Graph");

        const int rows = h / gridSize + 1;
        const int cols = w / gridSize + 1;

        lines = {};
        for (int i = 0; i <= rows; ++i) {
            const double y = offsetY + gridSize * i;

            std::vector<Node> currentLine = {};
            for (int j = 0; j <= cols; ++j) {
                const double x = offsetX + gridSize * j;
                Node n(x, y);
                gridGraph.addNode(n);
                currentLine.emplace_back(n);
            }
            lines.emplace_back(currentLine);
        }
    }

    bool removeIntersectingEdges(Graph& gridGraph, double gridSize, std::vector<std::vector<Node>> girdLines, double offsetX, double offsetY) {
        Log::d("[ MapView ] Remove intersecting segments edges");
        for (auto segment: segments) {
            //drawPath({segment.m_points[0].m_x, segment.m_points[0].m_y},{segment.m_points[1].m_x, segment.m_points[1].m_y},Qt::magenta);

            double gridX1 = segment.m_points[0].m_x - offsetX;
            double gridY1 = segment.m_points[0].m_y - offsetY;
            double gridX2 = segment.m_points[1].m_x - offsetX;
            double gridY2 = segment.m_points[1].m_y - offsetY;
            assert(gridX1 >= 0);
            assert(gridX2 >= 0);
            assert(gridY1 >= 0);
            assert(gridY2 >= 0);
            //std::cout << gridX1 << ", " << gridY1 <<  "), (" << gridX2 << ", " << gridY2 << ")" << std::endl;

            gridX1 = roundToGrid(gridX1, gridSize, gridX1 > gridX2);
            gridX2 = roundToGrid(gridX2, gridSize, gridX2 > gridX1);
            gridY1 = roundToGrid(gridY1, gridSize, gridY1 > gridY2);
            gridY2 = roundToGrid(gridY2, gridSize, gridY2 > gridY1);

            //std::cout << "("<< gridX1 << ", " << gridY1 <<  "), (" << gridX2 << ", " << gridY2 << ")" << std::endl;

            int lineX1 = gridX1 / gridSize;
            int lineX2 = gridX2 / gridSize;
            int lineY1 = gridY1 / gridSize;
            int lineY2 = gridY2 / gridSize;

            auto [minX, maxX] = std::minmax(lineX1, lineX2);
            auto [minY, maxY] = std::minmax(lineY1, lineY2);

            //std::cout << "("<<lineX1 << ", " << lineY1 <<  "), (" << lineX2 << ", " << lineY2 << ")" << std::endl;

            for (int j = minY; j <= maxY; ++j) {
                for (int i = minX; i <= maxX; ++i) {
                    assert(j < girdLines.size());
                    assert(i < girdLines[0].size());
                    auto n = girdLines[j][i];
                    auto neighbours = gridGraph.neighbours(n.id);
                    for (const auto neighbour: neighbours){
                        if (intersect(segment, {n.x, n.y}, {gridGraph.getNode(neighbour).x, gridGraph.getNode(neighbour).y})) {
                            gridGraph.removeEdge(n.id, neighbour);
                        }
                    }
                    drawLocation(n, "", Qt::green, N, 12.0);
                }
            }
        }
        return false;
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
        // constexpr double radius = 0.5;
        // constexpr auto color = Qt::red;
        // m_scene->addEllipse(
        //     {pos.x() - radius / 2, pos.y() - radius / 2, radius, radius},
        //     {color},
        //     {color, Qt::SolidPattern}
        // );
        //
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

    void drawTimeLabel(const int time) {
        if (m_timeCounter == nullptr) {
            m_timeCounter = new QGraphicsSimpleTextItem();
            m_scene->addItem(m_timeCounter);
            QFont font = m_timeCounter->font();
            font.setBold(false);
            font.setPointSize(12);
            m_timeCounter->setFont(font);
            m_timeCounter->setBrush(Qt::black);
            m_timeCounter->setPos(width() - 40, 20);
        }
        m_timeCounter->setText(QString::fromStdString(std::to_string(time)));
    }

    void drawStateLabel(Robot *robot) {
        if (m_stateLabel == nullptr) {
            m_stateLabel = new QGraphicsSimpleTextItem();
            m_scene->addItem(m_stateLabel);
            QFont font = m_stateLabel->font();
            font.setBold(false);
            font.setPointSize(12);
            m_stateLabel->setFont(font);
            m_stateLabel->setPos(width() - 140, 20);
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
            {Qt::transparent},
            {color, Qt::SolidPattern}
        );
        point->setZValue(zValue);

        QGraphicsSimpleTextItem *const eventLabel = m_scene->addSimpleText(QString::fromStdString(label));
        const double w = eventLabel->boundingRect().width();
        switch (labelPos) {
            case N: eventLabel->setPos(n.x - w, n.y - 2 * radius);
                break;
            case S: eventLabel->setPos(n.x - w, n.y + 1 * radius);
                break;
            case W: eventLabel->setPos(n.x - w - 2 * radius, n.y);
                break;
            case O: eventLabel->setPos(n.x - w + 2 * radius, n.y);
                break;
        }
        eventLabel->setZValue(Z_LOCATION + 1);
    }

    double roundToGrid(double value, double gridSize, bool up = true) {
        if (up) {
            return std::ceil(value / gridSize) * gridSize;
        }
        return std::floor(value / gridSize) * gridSize;
    }

    void drawPath(const Node &from, const Node &to, const QColor &color = Qt::black) const {
        QPen pen;
        pen.setWidthF(1.0);
        pen.setColor(color);
        const auto line = m_scene->addLine({from.x, from.y, to.x, to.y}, pen);
        line->setZValue(Z_DRIVE);
        line->setPen(pen);
    }

    void drawGraph(Graph &graph) {
        auto nodes = graph.allNodes();
        for (auto [id, node]: graph.allNodes()) {
            drawLocation(node, "", Qt::red, N, 7.0);
            //drawLocation(node, std::to_string(id), Qt::darkGray, N, 0.4);
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
            if (auto *escortEvent = dynamic_cast<MeetingEvent *>(ev->getValue())) {
                Node node = m_model.getGraph().getNode(escortEvent->getDestination());
                drawLocation(node, std::to_string(escortEvent->getPersonId()), Helper::color(ROBOT_STATE::MEETING), S);
            }
        }
    }

    void drawRobot(Robot *robot) {
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
