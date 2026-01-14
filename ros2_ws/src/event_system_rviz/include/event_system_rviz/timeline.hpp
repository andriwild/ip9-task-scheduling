#pragma once

#include <qgraphicsitem.h>
#include <qnamespace.h>

#include <QGraphicsView>
#include <QPushButton>
#include <QResizeEvent>
#include <QWheelEvent>
#include <cmath>
#include <vector>

#include "timeline_types.hpp"

constexpr int TIMELINE_HEIGHT = 350;
constexpr int TIMELINE_SCENE_MARGIN = 50;

constexpr int Y_LINE_POS = 200;
constexpr int LABEL_OFFSET = 20;
constexpr int X_LINE_OFFSET = TIMELINE_SCENE_MARGIN;
constexpr int MARKER_HEIGHT = 20;

constexpr int Z_TIMELINE = 100;
constexpr int Z_STATE_LINE = 50;
constexpr int Z_PLAN_LINE = 80;
constexpr int Z_MARKER = 50;

struct VisualEvent {
    int time;
    QString label;
    QString type;
};

struct VisualAppointment {
    std::shared_ptr<des::Appointment> appt;
    int startTime;
};

struct VisualStateBlock {
    int startTime;
    int endTime;
    int type;
};

class Timeline final : public QGraphicsView {
    Q_OBJECT

    QGraphicsScene * m_scene;
    const int m_simStartTime;
    const int m_simEndTime;
    int m_duration;

    double m_pixelsPerSecond;

    std::vector<VisualAppointment> m_appointments;
    std::vector<VisualEvent> m_events;

    std::vector<VisualStateBlock> m_states;
    VisualStateBlock m_currentOpenState;

    QPushButton * m_btnZoomIn;
    QPushButton * m_btnZoomOut;

public:
    explicit Timeline(int start, int end, double pixelsPerSecond = 0.025)
        : QGraphicsView(),
          m_simStartTime(start),
          m_simEndTime(end),
          m_pixelsPerSecond(pixelsPerSecond) {
        m_duration = m_simEndTime - m_simStartTime;
        m_scene = new QGraphicsScene(this);

        setDragMode(ScrollHandDrag);
        setRenderHint(QPainter::Antialiasing);

        setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);

        setScene(m_scene);
        centerOn(0, 0);

        setupButtons();
        updateScene();
    }

    void addMeetingPlan(std::shared_ptr<des::Appointment> appt, int startTime) {
        m_appointments.push_back({appt, startTime});
        drawMeetingPlan(appt, startTime);
    }

    void clear() {
        m_appointments.clear();
        m_events.clear();
        m_states.clear();
        m_currentOpenState = VisualStateBlock();
        m_currentOpenState.endTime = -1;
        updateScene();
    }

public slots:
    void handleLog(int time, QString message) {
        // m_events.push_back({time, message, "LOG"});
        // drawEventMarker({time, message, "LOG"});
    }

    void handleMove(int time, QString location) {
        QString label = "Moved: " + location;
        m_events.push_back({time, label, "MOVE"});
        drawEventMarker({time, label, "MOVE"});
    }

    void handleStateChange(int time, int newState) {
        if (m_currentOpenState.endTime == -1) {
            m_currentOpenState.endTime = time;
            m_states.push_back(m_currentOpenState);
        }

        m_currentOpenState.startTime = time;
        m_currentOpenState.endTime = -1;
        m_currentOpenState.type = newState;

        viewport()->update();
    }

    void handleReset() {
        clear();
    }

    void zoomIn() { applyZoom(1.5); }
    void zoomOut() { applyZoom(0.66); }

protected:
    void drawBackground(QPainter * painter, const QRectF& rect) override {
        // state line
        int barHeight = 10;
        int barY = Y_LINE_POS;

        auto getColor = [](int type) -> QColor {
            switch (static_cast<des::RobotStateType>(type)) {
                case des::RobotStateType::IDLE:
                    return Qt::lightGray;
                case des::RobotStateType::MOVING:
                    return QColor(100, 200, 100);
                case des::RobotStateType::ACCOMPANY:
                    return QColor(200, 150, 50);
                case des::RobotStateType::CHARGING:
                    return Qt::yellow;
                case des::RobotStateType::SEARCHING:
                    return QColor(200, 100, 100);
                case des::RobotStateType::CONVERSATE:
                    return QColor(180, 215, 230);
                default:
                    return Qt::gray;
            }
        };

        for (const auto& block : m_states) {
            if (timeToX(block.endTime) < rect.left()) {
                continue;
            }
            if (timeToX(block.startTime) > rect.right()) {
                continue;
            }

            double x1 = timeToX(block.startTime);
            double x2 = timeToX(block.endTime);

            painter->fillRect(QRectF(x1, barY, x2 - x1, barHeight), getColor(block.type));
        }

        if (m_currentOpenState.endTime != -1) {
            double x1 = timeToX(m_currentOpenState.startTime);
            double x2 = timeToX(m_simEndTime);
            painter->fillRect(QRectF(x1, barY, x2 - x1, barHeight), getColor(m_currentOpenState.type));
        }

        // baseline
        QPen axisPen(Qt::black, 2);
        painter->setPen(axisPen);
        double startX = std::max(rect.left(), timeToX(m_simStartTime));
        double endX = std::min(rect.right(), timeToX(m_simEndTime));

        if (startX < endX) {
            painter->drawLine(QLineF(startX, Y_LINE_POS, endX, Y_LINE_POS));
        }

        double tStart = xToTime(rect.left());
        double tEnd = xToTime(rect.right());

        int loopStart = std::max(m_simStartTime, static_cast<int>(std::floor(tStart)));
        int loopEnd = std::min(m_simEndTime, static_cast<int>(std::ceil(tEnd)));

        const double MIN_TICK_PX = 10.0;
        const double MIN_LABEL_PX = 80.0;

        const std::vector<int> intervals = {1, 2, 5, 10, 30, 60, 120, 300,
                                            600, 1800, 3600, 7200, 14400, 21600, 43200};

        int tickStep = 3600;
        for (int interval : intervals) {
            if (interval * m_pixelsPerSecond >= MIN_TICK_PX) {
                tickStep = interval;
                break;
            }
        }

        int labelStep = tickStep;
        for (int interval : intervals) {
            if (interval < tickStep) {
                continue;
            }
            if (interval * m_pixelsPerSecond >= MIN_LABEL_PX) {
                labelStep = interval;
                break;
            }
        }

        bool showSeconds = (labelStep < 60);

        int firstTick = loopStart - (loopStart % tickStep);
        if (firstTick < loopStart) {
            firstTick += tickStep;
        }

        QFont font = painter->font();
        font.setPointSize(8);
        painter->setFont(font);

        for (int time = firstTick; time <= loopEnd; time += tickStep) {
            double x = timeToX(time);

            bool isLabel = (time % labelStep == 0);

            if (isLabel) {
                painter->setPen(QPen(Qt::black, 2));
                painter->drawLine(QLineF(x, Y_LINE_POS + 5, x, Y_LINE_POS - 5));

                std::string timeStr = des::toHumanReadableTime(time, showSeconds);
                QString qs = QString::fromStdString(timeStr);

                QFontMetrics fm(font);
                int w = fm.horizontalAdvance(qs);
                painter->drawText(static_cast<int>(x) - w / 2, Y_LINE_POS + LABEL_OFFSET + 10, qs);
            } else {
                painter->setPen(QPen(Qt::gray, 1));
                painter->drawLine(QLineF(x, Y_LINE_POS + 3, x, Y_LINE_POS - 3));
            }
        }
    }

    void resizeEvent(QResizeEvent * event) override {
        QGraphicsView::resizeEvent(event);
        positionButtons();
    }

    void wheelEvent(QWheelEvent * event) override {
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

    void positionButtons() {
        int margin = 20;
        int x = width() - margin - m_btnZoomIn->width();
        int y = margin;
        m_btnZoomIn->move(x, y);
        m_btnZoomOut->move(x, y + m_btnZoomIn->height() + 5);
    }

    void applyZoom(double factor) {
        viewport()->setUpdatesEnabled(false);

        QPointF viewCenter = viewport()->rect().center();
        QPointF sceneCenter = mapToScene(viewCenter.toPoint());

        double timeFromStart = (sceneCenter.x() - X_LINE_OFFSET) / m_pixelsPerSecond;

        double newScale = m_pixelsPerSecond * factor;

        if (newScale >= 0.01 && newScale <= 2000.0) {
            m_pixelsPerSecond = newScale;

            updateScene();

            double newCenterX = X_LINE_OFFSET + (timeFromStart * m_pixelsPerSecond);
            centerOn(newCenterX, sceneCenter.y());
        }

        viewport()->setUpdatesEnabled(true);
    }

    void updateScene() {
        m_scene->clear();

        double totalWidth = m_duration * m_pixelsPerSecond;
        m_scene->setSceneRect(-TIMELINE_SCENE_MARGIN, 0, totalWidth + (TIMELINE_SCENE_MARGIN * 2),
                              TIMELINE_HEIGHT);

        for (const auto& item : m_appointments) {
            drawMeetingPlan(item.appt, item.startTime);
        }
        for (const auto& evt : m_events) {
            drawEventMarker(evt);
        }
    }

    double timeToX(int time) const {
        return X_LINE_OFFSET + (time - m_simStartTime) * m_pixelsPerSecond;
    }

    double xToTime(double x) const {
        return ((x - X_LINE_OFFSET) / m_pixelsPerSecond) + m_simStartTime;
    }

    void drawEventMarker(const VisualEvent& evt, QColor color = Qt::darkGreen) {
        double x = timeToX(evt.time);

        int lineTop = Y_LINE_POS - MARKER_HEIGHT;

        auto line = m_scene->addLine(x, Y_LINE_POS, x, lineTop, {color, 2});
        line->setZValue(Z_MARKER);

        auto text = m_scene->addText(evt.label);
        QFont f = text->font();
        f.setPointSize(8);
        text->setFont(f);
        text->setDefaultTextColor(color);
        text->setRotation(-60);
        text->setPos(x - 12, lineTop - 5);
        text->setZValue(Z_MARKER);
    }

    void drawMeetingPlan(std::shared_ptr<des::Appointment> appt, int startTime) {
        double startX = timeToX(startTime);
        double meetingX = timeToX(appt.get()->appointmentTime);
        double durationWidth = meetingX - startX;

        if (durationWidth < 0) {
            durationWidth = 0;
        }

        auto rect = m_scene->addRect(startX, Y_LINE_POS, durationWidth, -MARKER_HEIGHT, QPen(Qt::NoPen),
                                     QBrush(QColor(100, 100, 100, 50)));
        rect->setZValue(Z_PLAN_LINE);

        QString labelText = QString::fromStdString(appt.get()->description + " (" + appt.get()->personName + ")");
        drawEventMarker({appt.get()->appointmentTime, labelText, "MEETING"}, Qt::red);
    }
};
