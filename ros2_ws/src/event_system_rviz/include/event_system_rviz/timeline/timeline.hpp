#pragma once

#include <qgraphicsitem.h>
#include <qnamespace.h>

#include <QGraphicsView>
#include <QPushButton>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>

#include "event_system_rviz/timeline/drive_track.hpp"
#include "event_system_rviz/timeline/timeline_types.hpp"
#include "event_system_rviz/timeline/timeline_track.hpp"
#include "event_system_rviz/timeline/marker_track.hpp"
#include "event_system_rviz/timeline/state_track.hpp"
#include "event_system_rviz/timeline/battery_track.hpp"

// Constants
constexpr int SCENE_MARGIN  = 50;
constexpr int LABEL_OFFSET  = 20;
constexpr int MARKER_HEIGHT = 20;
constexpr int TRACK_GAP     = 20;
constexpr int X_LINE_OFFSET = SCENE_MARGIN;

class Timeline final : public QGraphicsView {
    Q_OBJECT

    QGraphicsScene * m_scene;
    int m_simStartTime;
    int m_simEndTime;
    int m_duration;

    double m_pixelsPerSecond;

    // Tracks
    std::shared_ptr<MarkerTrack> m_markerTrack;
    std::shared_ptr<StateTrack> m_stateTrack;
    std::shared_ptr<BatteryTrack> m_batteryTrack;
    std::shared_ptr<DriveTrack> m_driveTrack;
    std::vector<ITimelineTrack*> m_tracks;

    QPushButton* m_btnZoomIn;
    QPushButton* m_btnZoomOut;

public:
    explicit Timeline(const double pixelsPerSecond = 0.025):
        QGraphicsView(),
        m_pixelsPerSecond(pixelsPerSecond) 
    {
        m_duration = m_simEndTime - m_simStartTime;
        m_scene = new QGraphicsScene(this);
        
        // Initialize tracks
        m_markerTrack  = std::make_shared<MarkerTrack>(150);
        m_driveTrack   = std::make_shared<DriveTrack>(10);
        m_stateTrack   = std::make_shared<StateTrack>(20);
        m_batteryTrack = std::make_shared<BatteryTrack>(100);
        
        m_tracks.push_back(m_markerTrack.get());
        m_tracks.push_back(m_stateTrack.get());
        m_tracks.push_back(m_driveTrack.get());
        m_tracks.push_back(m_batteryTrack.get());

        setDragMode(ScrollHandDrag);
        setRenderHint(QPainter::Antialiasing);

        setViewportUpdateMode(SmartViewportUpdate);

        setScene(m_scene);
        centerOn(0, 0);

        setupButtons();
        updateScene();
    }

    void setRange(const int start, const int end) {
        m_simStartTime = start;
        m_simEndTime   = end;
        m_duration     = m_simEndTime - m_simStartTime;
        updateScene();
    }

    void addMeetingPlan(const std::shared_ptr<des::Appointment> &appointment, const int startTime) const {
        m_markerTrack->addMeetingPlan(appointment, startTime);
        updateScene();
    }

    void clear() const {
        for(const auto track : m_tracks) {
            track->clear();
        }
        updateScene();
    }

public slots:
    void handleStateChange(const int time, const int newState, const des::BatteryProps &batStats) const {
        m_stateTrack->handleStateChange(time, newState);
        m_batteryTrack->handleStateChange(time, batStats);
        updateScene();
    }

    void handleReset() const { clear(); }

    void handleEvent(const int time, const VisualEvent &ve, const bool isDriving) const {
        m_markerTrack->handleEvent(time, ve);
        m_driveTrack->handleStateChange(time, isDriving);
        updateScene(); // Events need scene update to draw items
    }

    void handleBattery(const int time, const double soc, const double capacity) const {}

    void zoomIn() { applyZoom(1.5); }
    void zoomOut() { applyZoom(0.66); }

protected:
    void drawBackground(QPainter* painter, const QRectF& rect) override {
        const double axisY = m_markerTrack->getHeight();
        drawTimeAxis(painter, rect, axisY);
    }

    void resizeEvent(QResizeEvent * event) override {
        QGraphicsView::resizeEvent(event);
        positionButtons();
    }

    void wheelEvent(QWheelEvent* event) override {
        if (event->modifiers() & Qt::ControlModifier) {
            const double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
            applyZoom(factor);
            event->accept();
        } else {
            QGraphicsView::wheelEvent(event);
        }
    }

private:
    void setupButtons() {
        m_btnZoomIn = new QPushButton("+", this);
        m_btnZoomIn->setCursor(Qt::ArrowCursor);
        m_btnZoomIn->setToolTip("Zoom In");
        m_btnZoomIn->setStyleSheet(
            "QPushButton { font-weight: bold; font-size: 16px; background-color: rgba(255, 255, 255, "
            "200); border: 1px solid #999; border-radius: 4px; } QPushButton:hover { background-color: "
            "white; }");
        connect(m_btnZoomIn, &QPushButton::clicked, this, &Timeline::zoomIn);

        m_btnZoomOut = new QPushButton("-", this);
        m_btnZoomOut->setCursor(Qt::ArrowCursor);
        m_btnZoomOut->setToolTip("Zoom Out");
        m_btnZoomOut->setStyleSheet(
            "QPushButton { font-weight: bold; font-size: 16px; background-color: rgba(255, 255, 255, "
            "200); border: 1px solid #999; border-radius: 4px; } QPushButton:hover { background-color: "
            "white; }");
        connect(m_btnZoomOut, &QPushButton::clicked, this, &Timeline::zoomOut);

        m_btnZoomIn->resize(30, 30);
        m_btnZoomOut->resize(30, 30);
    }

    void positionButtons() const {
        constexpr int margin = 20;
        const int x = width() - margin - m_btnZoomIn->width();
        const int y = margin;
        m_btnZoomIn->move(x, y);
        m_btnZoomOut->move(x, y + m_btnZoomIn->height() + 5);
    }

    void applyZoom(const double factor) {
        viewport()->setUpdatesEnabled(false);

        const QPointF viewCenter  = viewport()->rect().center();
        const QPointF sceneCenter = mapToScene(viewCenter.toPoint());

        const double timeFromStart = (sceneCenter.x() - X_LINE_OFFSET) / m_pixelsPerSecond;
        const double newScale = m_pixelsPerSecond * factor;

        if (newScale >= 0.01 && newScale <= 2000.0) {
            m_pixelsPerSecond = newScale;
            updateScene();
            const double newCenterX = X_LINE_OFFSET + (timeFromStart * m_pixelsPerSecond);
            centerOn(newCenterX, sceneCenter.y());
        }
        viewport()->setUpdatesEnabled(true);
    }

    void updateScene() const {
        m_scene->clear();

        const double totalWidth = m_duration * m_pixelsPerSecond;
        double totalHeight = 0;
        for(const auto& track : m_tracks) {
            totalHeight += track->getHeight();
        }

        m_scene->setSceneRect( -SCENE_MARGIN, 0, totalWidth + (SCENE_MARGIN * 2), totalHeight );

        double currentY = 0.0;
        int index = 0;
        std::vector gaps(m_tracks.size(), TRACK_GAP);
        gaps[0] = 2 * TRACK_GAP;

        for (const auto& track : m_tracks) {
            track->updateScene(m_scene, m_pixelsPerSecond, m_simStartTime, X_LINE_OFFSET, currentY);

            // track label on the left side
            QString name = QString::fromStdString(track->getName());
            QFont font;
            font.setPointSize(10);
            QFontMetrics fm(font);
            const int textWidth = fm.horizontalAdvance(name);
            constexpr int padding = 5;

            constexpr double leftBoundary = -SCENE_MARGIN;
            constexpr double rightBoundary = X_LINE_OFFSET - padding;
            
            if (rightBoundary - textWidth >= leftBoundary) {
                const auto textItem = m_scene->addText(name, font);
                textItem->setDefaultTextColor(Qt::black);
                
                const double textX = rightBoundary - textItem->boundingRect().width();
                const double textY = currentY + (track->getHeight() / 2.0) - (textItem->boundingRect().height() / 2.0);
                textItem->setPos(textX, textY);
            }
            currentY += track->getHeight() + gaps[index++];
        }
        viewport()->update();
    }

    [[nodiscard]] double timeToX(const int time) const {
        return X_LINE_OFFSET + (time - m_simStartTime) * m_pixelsPerSecond;
    }

    [[nodiscard]] double xToTime(const double x) const {
        return ((x - X_LINE_OFFSET) / m_pixelsPerSecond) + m_simStartTime;
    }
    
    void drawTimeAxis(QPainter* painter, const QRectF& rect, const double yPos) const {
        const QPen axisPen(Qt::black, 2);
        painter->setPen(axisPen);
        const double startX = std::max(rect.left() , timeToX(m_simStartTime));
        const double endX   = std::min(rect.right(), timeToX(m_simEndTime));

        painter->drawLine(QLineF(startX, yPos, endX, yPos));

        const double tStart = xToTime(rect.left());
        const double tEnd   = xToTime(rect.right());

        const int loopStart = std::max(m_simStartTime, static_cast<int>(std::floor(tStart)));
        const int loopEnd   = std::min(m_simEndTime  , static_cast<int>(std::ceil(tEnd)));

        constexpr double MIN_TICK_PX  = 10.0;
        constexpr double MIN_LABEL_PX = 80.0;

        const std::vector intervals = {
            1, 2, 5, 10, 30, 60, 120, 300, 600, 1800, 3600, 7200, 14400, 21600, 43200
        };

        int tickStep = 3600;
        for (const int interval : intervals) {
            if (interval * m_pixelsPerSecond >= MIN_TICK_PX) {
                tickStep = interval;
                break;
            }
        }

        int labelStep = tickStep;
        for (const int interval : intervals) {
            if (interval < tickStep) {
                continue;
            }
            if (interval * m_pixelsPerSecond >= MIN_LABEL_PX) {
                labelStep = interval;
                break;
            }
        }

        const bool showSeconds = (labelStep < 60);

        int firstTick = loopStart - (loopStart % tickStep);
        if (firstTick < loopStart) {
            firstTick += tickStep;
        }

        // time labels
        QFont font = painter->font();
        const QFontMetrics fm(font);
        font.setPointSize(8);
        painter->setFont(font);

        for (int time = firstTick; time <= loopEnd; time += tickStep) {
            const double x = timeToX(time);

            const bool isLabel = time % labelStep == 0;

            if (isLabel) {
                painter->setPen(QPen(Qt::black, 2));
                painter->drawLine(QLineF(x, yPos + 5, x, yPos - 5));

                std::string timeStr = des::toHumanReadableTime(time, showSeconds);
                QString qs = QString::fromStdString(timeStr);

                int w = fm.horizontalAdvance(qs);
                painter->drawText(static_cast<int>(x) - w / 2, yPos + LABEL_OFFSET + 10, qs);
            } else {
                painter->setPen(QPen(Qt::gray, 1));
                painter->drawLine(QLineF(x, yPos + 3, x, yPos - 3));
            }
        }
    }
};
