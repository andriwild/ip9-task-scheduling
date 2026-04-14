#pragma once

#include "event_system_rviz/timeline/timeline_track.hpp"
#include "event_system_rviz/timeline/timeline_types.hpp"
#include <QMultiMap>
#include <QGraphicsTextItem>
#include <vector>

class MarkerTrack final: public ITimelineTrack {
    QMultiMap<int, VisualEvent> m_events; // multiple events at the same time possible
    std::vector<VisualAppointment> m_appointments;
    int m_height;
    
    const int MARKER_HEIGHT = 20;
    const int Z_MARKER      = 50;
    const int Z_PLAN_LINE   = 80;

public:
    explicit MarkerTrack(const int height = 150) : m_height(height) {}

    std::string getName() const override { return ""; }

    void handleEvent(const int time, const VisualEvent& ve) {
        m_events.insert(time, ve);
    }

    void addMeetingPlan(const VisualAppointment& appt) {
        m_appointments.push_back(appt);
    }

    double getHeight() const override {
        return m_height;
    }
    
    void clear() override {
        m_events.clear();
        m_appointments.clear();
    }

    void updateScene(QGraphicsScene* scene, const double pixelsPerSecond, const int simStartTime, const double xOffset, const double yBase) override {
        const double yAxis = yBase + m_height;
        const TimelineTransformer tf { pixelsPerSecond, simStartTime, xOffset };

        // Draw Appointments
        for (const auto& appt : m_appointments) {
            const double startX = tf.toX(appt.startTime);
            const double meetingX = tf.toX(appt.scheduledTime);
            double durationWidth = meetingX - startX;

            if (durationWidth < 0) durationWidth = 0;

            const auto rect = scene->addRect(
                startX,
                yAxis,
                durationWidth,
                -(MARKER_HEIGHT / 2),
                QPen(Qt::NoPen),
                QBrush(QColor(100, 100, 100, 50))
            );
            rect->setZValue(Z_PLAN_LINE);

            const QString labelText = QString::fromStdString(
                std::format("Mission {}: {} ({})", appt.id, appt.description, appt.primaryLabel)
                );
            drawMeetingMarker(scene, appt.scheduledTime, labelText, yAxis, tf);
        }

        // Draw Events
        QList<int> eventTimes = m_events.uniqueKeys();
        for (int t : eventTimes) {
            QList<VisualEvent> eventsOfTime = m_events.values(t);
            int counter = 0;
            for(const auto& ev : eventsOfTime) {
                drawEventMarker(scene, t, ev, counter, yAxis, tf);
                counter++;
            }
        }
    }

private:
   void drawMeetingMarker(
       QGraphicsScene* scene,
       const int time,
       const QString& label,
       const double yAxis,
       const TimelineTransformer& tf
       ) const {
        const double x = tf.toX(time);
        QColor color = Qt::red;

        const int lineTop = yAxis - MARKER_HEIGHT;

        const qreal size = MARKER_HEIGHT;
        const qreal halfW = size / 4.0;
        const qreal halfH = size / 2.0;
        QPolygonF diamondShape;
        diamondShape << QPointF(x, yAxis)
            << QPointF(x + halfW, yAxis - halfH)
            << QPointF(x, yAxis - size)
            << QPointF(x - halfW, yAxis - halfH);

        const auto diamondItem = scene->addPolygon(diamondShape, {color, 1}, {color});
        diamondItem->setZValue(Z_MARKER);

        const QString eventLabel = label
            +  " - (" 
            + QString::fromStdString(des::toHumanReadableTime(time, true)) 
            + ")";

        const auto text = scene->addText(eventLabel);
        QFont f = text->font();
        f.setPointSize(8);
        text->setFont(f);
        text->setDefaultTextColor(color);
        text->setRotation(-60);
        text->setPos(x - 12, lineTop - 7);
        text->setZValue(Z_MARKER);
    }

   void drawEventMarker(
       QGraphicsScene *scene,
       const int time,
       const VisualEvent &evt,
       const int etage,
       const double yAxis,
       const TimelineTransformer &tf
   ) const {
       const double x = tf.toX(time);

       constexpr double verticalGap = 20;
       const double verticalOffset = etage * MARKER_HEIGHT + etage * verticalGap;

       const double yLineTop = yAxis - MARKER_HEIGHT - verticalOffset;
       const double yLineBottom = yAxis - verticalOffset;

       QColor color = getEventColor(evt.type);

       const auto line = scene->addLine(x, yLineBottom, x, yLineTop, {color, 2});
       line->setZValue(Z_MARKER);

       const QString eventLabel = evt.label
                                  + " - ("
                                  + QString::fromStdString(des::toHumanReadableTime(time, true))
                                  + ")";

       const auto text = scene->addText(eventLabel);
       QFont f = text->font();
       f.setPointSize(8);
       text->setFont(f);
       text->setDefaultTextColor(color);
       text->setRotation(-60);
       text->setPos(x - 12, yLineTop - 7);
       text->setZValue(Z_MARKER);
   }
};