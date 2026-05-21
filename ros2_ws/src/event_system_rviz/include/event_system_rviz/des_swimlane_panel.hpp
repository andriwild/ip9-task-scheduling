#pragma once

#include <qobjectdefs.h>

#include <QColor>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QListWidget>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>
#include <functional>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <vector>

#include "event_system_msgs/msg/system_config.hpp"
#include "event_system_msgs/msg/timeline_event.hpp"
#include "event_system_msgs/msg/timeline_meeting.hpp"
#include "event_system_msgs/msg/timeline_reset.hpp"
#include "event_system_msgs/msg/timeline_state_change.hpp"

namespace des_swimlane_panel {

// ---------------- Data records ----------------

struct EventRecord {
    int time;
    quint8 type;
    QString label;
    QString color;
    bool isDriving;
    bool isCharging;
    int missionId = -1;
};

struct StateChangeRecord {
    int time;
    quint8 state;       // structural category (IDLE/MISSION/CHARGING/RETURNING)
    QString name;       // plugin-supplied state name for display
    float soc;
};

struct MeetingRecord {
    int id;
    quint8 missionState;
    int appointmentTime;
    int startTime;
    QString personName;
    QString roomName;
    QString description;
    quint8 executionMode = 1;  // default BACKGROUND
};

// ---------------- Main view: wheel-scroll, ctrl-wheel-zoom ----------------

class SwimlaneView : public QGraphicsView {
public:
    explicit SwimlaneView(QWidget* parent = nullptr);
    void setOnViewportChanged(std::function<void()> cb) { m_onViewportChanged = std::move(cb); }
    void resetZoom();
    qreal xScale() const;

protected:
    void wheelEvent(QWheelEvent* e) override;
    void scrollContentsBy(int dx, int dy) override;
    void keyPressEvent(QKeyEvent* e) override;

private:
    void zoomXAt(qreal factor, QPoint viewportAnchor);
    std::function<void()> m_onViewportChanged;
};

// ---------------- Minimap: small overview with brushable range ----------------

class MinimapView : public QGraphicsView {
public:
    explicit MinimapView(QWidget* parent = nullptr);

    void setSimRange(int tMin, int tMax);
    void setEventTimes(const std::vector<int>& times);
    void setBrush(int tBrushStart, int tBrushEnd);
    void setOnBrushChanged(std::function<void(int, int)> cb);

protected:
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;

private:
    void rebuildHistogram();
    int xToTime(int x) const;

    QGraphicsScene m_scene;
    QGraphicsRectItem* m_brushItem = nullptr;
    std::vector<int> m_times;
    int m_tMin = 0;
    int m_tMax = 0;
    int m_brushStart = 0;
    int m_brushEnd = 0;
    enum class Drag { None, Brush, LeftEdge, RightEdge } m_drag = Drag::None;
    int m_dragOffset = 0;
    std::function<void(int, int)> m_onBrushChanged;
};

// ---------------- Panel ----------------

class DesSwimlanePanel : public rviz_common::Panel {
    Q_OBJECT
public:
    explicit DesSwimlanePanel(QWidget* parent = nullptr);
    ~DesSwimlanePanel() override;
    void onInitialize() override;

private:
    // ROS callbacks (called on Qt thread via invokeMethod)
    void onEvent(const event_system_msgs::msg::TimelineEvent::SharedPtr msg);
    void onStateChange(const event_system_msgs::msg::TimelineStateChange::SharedPtr msg);
    void onMeeting(const event_system_msgs::msg::TimelineMeeting::SharedPtr msg);
    void onReset(const event_system_msgs::msg::TimelineReset::SharedPtr msg);

    // Rendering
    void scheduleRedraw();
    void redraw();
    void rebuildFilterList();

    // Derived buckets ---------------------------------------------------------
    struct ComputedMission {
        int id;
        int startTime;
        int endTime;       // -1 if still running
        QString personName;
        QString roomName;
        QString description;
        QColor color;
    };

    // Helpers for grouping
    QColor pickPersonColor(const QString& name);
    static QColor parseColor(const QString& s, const QColor& fallback);
    static QString eventTypeName(quint8 type);
    static QString stateName(quint8 state);

    // Data ------------------------------------------------------------------
    std::vector<EventRecord> m_events;
    std::vector<StateChangeRecord> m_states;
    std::map<int, MeetingRecord> m_meetings;  // by id, latest wins
    // Battery low-threshold as 0..1 fraction (msg carries it in percent 0..100).
    // Updated from every TimelineStateChange so config changes propagate.
    float m_lowThreshold = 0.2f;

    // Widgets ---------------------------------------------------------------
    QGraphicsScene* m_scene = nullptr;
    SwimlaneView* m_view = nullptr;
    MinimapView* m_minimap = nullptr;
    QListWidget* m_filterList = nullptr;
    QWidget* m_labelColumn = nullptr;
    QVBoxLayout* m_labelLayout = nullptr;

    // ROS -------------------------------------------------------------------
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<event_system_msgs::msg::TimelineEvent>::SharedPtr m_subEvent;
    rclcpp::Subscription<event_system_msgs::msg::TimelineStateChange>::SharedPtr m_subStateChange;
    rclcpp::Subscription<event_system_msgs::msg::TimelineMeeting>::SharedPtr m_subMeeting;
    rclcpp::Subscription<event_system_msgs::msg::TimelineReset>::SharedPtr m_subReset;
    rclcpp::Subscription<event_system_msgs::msg::SystemConfig>::SharedPtr m_subConfig;

    // Range / view state ----------------------------------------------------
    int m_tMin = 0;
    int m_tMax = 0;
    int m_brushStart = 0;
    int m_brushEnd = 0;

    // Scene mapping (set in redraw, used by view/brush sync)
    int m_paddedTMin = 0;
    int m_paddedTMax = 1;
    qreal m_baseScaleX = 1.0;  // scene units per second (before view transform)
    int m_labelColumnPx = 140;

    // Throttled redraw + sync guards
    bool m_redrawScheduled = false;
    bool m_syncing = false;

    void syncBrushFromView();
    void syncViewFromBrush();
};

}  // namespace des_swimlane_panel
