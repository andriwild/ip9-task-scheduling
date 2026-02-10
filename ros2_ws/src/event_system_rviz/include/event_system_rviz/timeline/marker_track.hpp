#pragma once

#include "event_system_rviz/timeline/timeline_track.hpp"
#include "event_system_rviz/timeline/timeline_types.hpp"
#include <QMultiMap>
#include <QGraphicsTextItem>
#include <QGraphicsLineItem>
#include <vector>

class MarkerTrack : public ITimelineTrack {
    QMultiMap<int, VisualEvent> m_events; // multiple events at the same time possible
    std::vector<VisualAppointment> m_appointments;
    int m_height;
    
    const int MARKER_HEIGHT = 20;
    const int Z_MARKER      = 50;
    const int Z_PLAN_LINE   = 80;

public:
    MarkerTrack(int height = 150) : m_height(height) {}

    std::string getName() const override { return ""; }

    void handleEvent(int time, VisualEvent ve) {
        m_events.insert(time, ve);
    }

    void addMeetingPlan(std::shared_ptr<des::Appointment> appt, int startTime) {
        m_appointments.push_back({appt, startTime});
    }

    double getHeight() const override {
        return m_height;
    }
    
    void clear() override {
        m_events.clear();
        m_appointments.clear();
    }

    void updateScene(QGraphicsScene* scene, double pixelsPerSecond, int simStartTime, double xOffset, double yBase) override {
        double yAxis = yBase + m_height;

        auto timeToX = [&](int time) {
            return xOffset + (time - simStartTime) * pixelsPerSecond;
        };

        // Draw Appointments
        for (const auto& item : m_appointments) {
            double startX = timeToX(item.startTime);
            double meetingX = timeToX(item.appt->appointmentTime);
            double durationWidth = meetingX - startX;

            if (durationWidth < 0) durationWidth = 0;

            auto rect = scene->addRect(
                startX, 
                yAxis, 
                durationWidth, 
                static_cast<int>(-(MARKER_HEIGHT / 2)), 
                QPen(Qt::NoPen),
                QBrush(QColor(100, 100, 100, 50))
            );
            rect->setZValue(Z_PLAN_LINE);

            QString labelText = QString::fromStdString(item.appt->description + " (" + item.appt->personName + ")");
            drawMeetingMarker(scene, item.appt->appointmentTime, labelText, timeToX, yAxis);
        }

        // Draw Events
        QList<int> eventTimes = m_events.uniqueKeys();
        for (int t : eventTimes) {
            QList<VisualEvent> eventsOfTime = m_events.values(t);
            int counter = 0;
            for(const auto& ev : eventsOfTime) {
                drawEventMarker(scene, t, ev, counter, timeToX, yAxis);
                counter++;
            }
        }
    }

private:
   void drawMeetingMarker(QGraphicsScene* scene, int time, QString label, std::function<double(int)> timeToX, double yAxis) {
        double x = timeToX(time);
        QColor color = Qt::red;

        int lineTop = yAxis - MARKER_HEIGHT;

        qreal size = MARKER_HEIGHT;
        qreal halfW = size / 4.0; 
        qreal halfH = size / 2.0;
        QPolygonF diamondShape;
        diamondShape << QPointF(x, yAxis)
            << QPointF(x + halfW, yAxis - halfH)
            << QPointF(x, yAxis - size)
            << QPointF(x - halfW, yAxis - halfH);

        auto diamondItem = scene->addPolygon(diamondShape, {color, 1}, {color});
        diamondItem->setZValue(Z_MARKER);

        QString eventLabel = label 
            +  " - (" 
            + QString::fromStdString(des::toHumanReadableTime(time, true)) 
            + ")";

        auto text = scene->addText(eventLabel);
        QFont f = text->font();
        f.setPointSize(8);
        text->setFont(f);
        text->setDefaultTextColor(color);
        text->setRotation(-60);
        text->setPos(x - 12, lineTop - 7);
        text->setZValue(Z_MARKER);
    }

    void drawEventMarker(QGraphicsScene* scene, int time, const VisualEvent& evt, int etage, std::function<double(int)> timeToX, double yAxis) {
        double x = timeToX(time);

        double verticalGap = 20;
        double verticalOffset = etage * MARKER_HEIGHT + etage * verticalGap; 

        double yLineTop    = yAxis - MARKER_HEIGHT - verticalOffset;
        double yLineBottom = yAxis - verticalOffset;

        QColor color = getEventColor(evt.type);

        auto line = scene->addLine(x, yLineBottom, x, yLineTop, {color, 2});
        line->setZValue(Z_MARKER);

        QString eventLabel = evt.label 
            +  " - (" 
            + QString::fromStdString(des::toHumanReadableTime(time, true)) 
            + ")";

        auto text = scene->addText(eventLabel);
        QFont f = text->font();
        f.setPointSize(8);
        text->setFont(f);
        text->setDefaultTextColor(color);
        text->setRotation(-60);
        text->setPos(x - 12, yLineTop - 7);
        text->setZValue(Z_MARKER);
    }
};
