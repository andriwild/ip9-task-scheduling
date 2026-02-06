#pragma once

#include <QPainter>
#include <QGraphicsScene>
#include <QRectF>

class TimelineTrack {
public:
    virtual ~TimelineTrack() = default;
    virtual void draw(QPainter* painter, const QRectF& rect, double pixelsPerSecond, int simStartTime, double xOffset, double yBase) {}
    virtual void updateScene(QGraphicsScene* scene, double pixelsPerSecond, int simStartTime, double xOffset, double yBase) {}
    virtual double getHeight() const = 0;
    virtual void clear() = 0;
};
