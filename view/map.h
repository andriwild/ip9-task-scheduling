#pragma once

#include <qgraphicsitem.h>
#include <QGraphicsView>

#include "../util/map_loader.h"

class Map: public QGraphicsItemGroup {
    QGraphicsLineItem* m_directionLine = nullptr;
    std::vector<VisLib::Segment>& m_segments;
    std::vector<VisLib::Point>& m_points;

public:
    explicit Map(
        std::vector<VisLib::Segment>& segments,
        std::vector<VisLib::Point>& points,
        QGraphicsItem* parent = nullptr
        ) :
    QGraphicsItemGroup(parent),
    m_segments(segments),
    m_points(points) {
        drawMap();
    }

    QRectF boundingRect() const override {
        return childrenBoundingRect();
    }

    void drawMap() {
        QPen pen;
        pen.setWidthF(2.0);

        for (const auto s: m_segments) {
            QPointF p1 (s.m_points[0].m_x, s.m_points[0].m_y);
            QPointF p2 (s.m_points[1].m_x, s.m_points[1].m_y);

            auto* line = new QGraphicsLineItem({p1, p2}, this);
            line->setPen(pen);
        }

        double radius = 20.0;
        for (const auto p: m_points) {

            auto qPoint = new QGraphicsEllipseItem(
                {p.m_x * 100 - radius / 2, p.m_y * 100 - radius / 2, radius, radius}, this
                );

            QPen pointPen;
            pointPen.setWidthF(5.0);
            pointPen.setColor(Qt::blue);
            qPoint->setPen(pointPen);
            qPoint->setBrush({Qt::blue, Qt::SolidPattern});
        }
    }
};
