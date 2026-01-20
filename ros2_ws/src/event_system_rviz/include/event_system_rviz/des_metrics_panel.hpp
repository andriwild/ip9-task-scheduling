#pragma once

#include <QGridLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QtCharts/QChartView>
#include <QtCharts/QPieSeries>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "event_system_msgs/msg/metrics_report.hpp"

namespace des_metrics_panel {

class DesMetricsPanel : public rviz_common::Panel {
    Q_OBJECT

    public:
    explicit DesMetricsPanel(QWidget* parent = nullptr);
    ~DesMetricsPanel() override;
    void onInitialize() override;

private:
    void onMetricsReport(const event_system_msgs::msg::MetricsReport::SharedPtr msg);
    QString fmtTime(int s);

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<event_system_msgs::msg::MetricsReport>::SharedPtr m_subscriber;

    QtCharts::QChartView* m_chartView;
    QtCharts::QPieSeries* m_missionSeries;

    QLabel* m_lblRobotState;
    QLabel* m_lblMissionStats;
    QLabel* m_lblPerformance;
    QLabel* m_lblMovement;
};

}  // namespace des_metrics_panel
