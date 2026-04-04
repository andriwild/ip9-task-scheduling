#include <QGroupBox>
#include <QVBoxLayout>
#include <event_system_rviz/des_metrics_panel.hpp>
#include <iomanip>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <sstream>
#include "event_system_msgs/msg/metrics_report.hpp"

namespace des_metrics_panel {

DesMetricsPanel::DesMetricsPanel(QWidget* parent) : rviz_common::Panel(parent) {
    auto mainLayout = new QVBoxLayout(this);

    m_missionSeries = new QtCharts::QPieSeries();
    m_missionSeries->setHoleSize(0.35);
    m_missionSeries->append("On Time", 1);
    m_missionSeries->append("Late", 1);
    m_missionSeries->append("Failed", 1);

    m_missionSeries->slices().at(0)->setColor(Qt::green);
    m_missionSeries->slices().at(0)->setLabelVisible(true);
    m_missionSeries->slices().at(1)->setColor(QColor(255, 165, 0));
    m_missionSeries->slices().at(2)->setColor(Qt::red);

    auto chart = new QtCharts::QChart();
    chart->addSeries(m_missionSeries);
    chart->setTitle("Mission Performance");
    chart->legend()->setAlignment(Qt::AlignRight);

    m_chartView = new QtCharts::QChartView(chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);
    m_chartView->setMinimumHeight(250);
    mainLayout->addWidget(m_chartView);

    auto statsGroup = new QGroupBox("Statistics");
    auto grid = new QGridLayout(statsGroup);

    m_lblRobotState = new QLabel("Idle: 0s\nMoving: 0s\nSearching: 0s\nAccompany: 0s\nCharging: 0s");
    grid->addWidget(new QLabel("<b>Robot State:</b>"), 0, 0);
    grid->addWidget(m_lblRobotState, 1, 0);

    m_lblMissionStats = new QLabel("Total: 0\nOn Time: 0\nLate: 0\nFailed: 0\nCancelled: 0\nRejected: 0");
    grid->addWidget(new QLabel("<b>Missions:</b>"), 0, 1);
    grid->addWidget(m_lblMissionStats, 1, 1);

    m_lblPerformance = new QLabel("Avg Early: 0.0s\nAvg Late: 0.0s");
    grid->addWidget(new QLabel("<b>Performance:</b>"), 2, 0);
    grid->addWidget(m_lblPerformance, 3, 0);

    m_lblMovement = new QLabel("Distance: 0.0m\nUtilization: 0.0%");
    grid->addWidget(new QLabel("<b>General:</b>"), 2, 1);
    grid->addWidget(m_lblMovement, 3, 1);

    mainLayout->addWidget(statsGroup);
    mainLayout->addStretch();
}

void DesMetricsPanel::onInitialize() {
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = node_abstraction->get_raw_node();

    auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

    m_subscriber = m_node->create_subscription<event_system_msgs::msg::MetricsReport>(
        "/metrics_report", qos,
        [this](const event_system_msgs::msg::MetricsReport::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                this->onMetricsReport(msg);
            });
        });

    m_subReset = m_node->create_subscription<event_system_msgs::msg::TimelineReset>(
        "/timeline/reset", qos,
        [this](const event_system_msgs::msg::TimelineReset::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onReset(msg); });
        });
}

QString DesMetricsPanel::fmtTime(const int s) {
    const int m = s / 60;
    const int sec = s % 60;
    std::ostringstream oss;
    oss << m << "m " << std::setw(2) << std::setfill('0') << sec << "s";
    return QString::fromStdString(oss.str());
}

void DesMetricsPanel::onReset(const event_system_msgs::msg::TimelineReset::SharedPtr /*msg*/) {
    auto emptyReport = std::make_shared<event_system_msgs::msg::MetricsReport>();
    onMetricsReport(emptyReport);
}

void DesMetricsPanel::onMetricsReport(const event_system_msgs::msg::MetricsReport::SharedPtr msg) {
    // Indices: 0=OnTime, 1=Late, 2=Failed
    m_missionSeries->slices().at(0)->setValue(msg->missions_on_time);
    m_missionSeries->slices().at(0)->setLabel(QString("On Time: %1").arg(msg->missions_on_time));

    m_missionSeries->slices().at(1)->setValue(msg->missions_late);
    m_missionSeries->slices().at(1)->setLabel(QString("Late: %1").arg(msg->missions_late));

    int notSuccess = msg->missions_failed + msg->missions_cancelled + msg->missions_rejected;
    m_missionSeries->slices().at(2)->setValue(notSuccess);  // Merging for "Not Success" logic
    m_missionSeries->slices().at(2)->setLabel(QString("Failed/Rejected: %1").arg(notSuccess));

    QString stateText = QString("Idle: %1\nMoving: %2\nSearching: %3\nAccompany: %4\nCharging: %5")
                            .arg(fmtTime(msg->idle_time))
                            .arg(fmtTime(msg->moving_time))
                            .arg(fmtTime(msg->searching_time))
                            .arg(fmtTime(msg->accompany_time))
                            .arg(fmtTime(msg->charging_time));
    m_lblRobotState->setText(stateText);

    QString missionText = QString("Total: %1\nOn Time: %2\nLate: %3\nFailed: %4\nCancelled: %5\nRejected: %6")
                              .arg(msg->total_missions)
                              .arg(msg->missions_on_time)
                              .arg(msg->missions_late)
                              .arg(msg->missions_failed)
                              .arg(msg->missions_cancelled)
                              .arg(msg->missions_rejected);
    m_lblMissionStats->setText(missionText);

    QString perfText = QString("Avg Early: %1s\nAvg Late: %2s")
                           .arg(QString::number(msg->avg_early_arrival, 'f', 1))
                           .arg(QString::number(msg->avg_lateness, 'f', 1));
    m_lblPerformance->setText(perfText);

    QString moveText = QString("Distance: %1m\nUtilization: %2%")
                           .arg(QString::number(msg->total_distance, 'f', 2))
                           .arg(QString::number(msg->utilization, 'f', 1));
    m_lblMovement->setText(moveText);
}

}  // namespace des_metrics_panel

PLUGINLIB_EXPORT_CLASS(des_metrics_panel::DesMetricsPanel, rviz_common::Panel)
