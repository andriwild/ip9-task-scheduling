#pragma once

#include <QPainter>
#include <QGraphicsScene>
#include <QRectF>

class ITimelineTrack {
public:
    virtual ~ITimelineTrack() = default;
    virtual void updateScene(QGraphicsScene* scene, double pixelsPerSecond, int simStartTime, double xOffset, double yBase) {}
    virtual double getHeight() const = 0;
    virtual void clear() = 0;
};
