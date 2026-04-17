#pragma once

#include <QMouseEvent>
#include <QToolTip>
#include <QVBoxLayout>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QCategoryAxis>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "event_system_msgs/msg/timeline_event.hpp"
#include "event_system_msgs/msg/timeline_reset.hpp"

namespace des_occupancy_panel {

/// QChartView subclass that resets zoom on double-click
class InteractiveChartView : public QtCharts::QChartView {
    Q_OBJECT
public:
    using QtCharts::QChartView::QChartView;
protected:
    void mouseDoubleClickEvent(QMouseEvent* event) override {
        chart()->zoomReset();
        event->accept();
    }
};

class DesOccupancyPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit DesOccupancyPanel(QWidget* parent = nullptr);
    ~DesOccupancyPanel() override = default;
    void onInitialize() override;

private:
    void onTimelineEvent(const event_system_msgs::msg::TimelineEvent::SharedPtr msg);
    void onReset(const event_system_msgs::msg::TimelineReset::SharedPtr msg);

    std::pair<QString, QString> parseLabel(const QString& label) const;
    int roomIndex(const QString& room);
    QtCharts::QLineSeries* getOrCreateSeries(const QString& person, const QString& color);
    void connectSeriesSignals(QtCharts::QLineSeries* series);
    void connectLegendMarkers();
    void addRobotDeparture(int time);
    void addRobotArrival(const QString& room, int time);
    void addPersonDeparture(const QString& name, const QString& room, int time, const QString& color);
    void addPersonArrival(const QString& name, const QString& room, int time, const QString& color);
    void addPoint(const QString& name, const QString& room, int time, const QString& color);
    void updateXAxis();

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<event_system_msgs::msg::TimelineEvent>::SharedPtr m_subEvent;
    rclcpp::Subscription<event_system_msgs::msg::TimelineReset>::SharedPtr m_subReset;

    QtCharts::QChart* m_chart;
    InteractiveChartView* m_chartView;
    QtCharts::QValueAxis* m_axisX;
    QtCharts::QCategoryAxis* m_axisY;

    QMap<QString, QtCharts::QLineSeries*> m_seriesMap;
    QMap<QString, int> m_roomIndexMap;
    QMap<QString, int> m_lastY;

    double m_simStart = 0;
    double m_simEnd = 86400;
    int m_nextRoomIndex = 1;
};

}  // namespace des_occupancy_panel
