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

    // Pie chart visualises scheduled missions only — one slice per terminal
    // state. Slice indices follow the order below.
    m_missionSeries = new QtCharts::QPieSeries();
    m_missionSeries->setHoleSize(0.35);
    m_missionSeries->append("On Time", 1);
    m_missionSeries->append("Late", 1);
    m_missionSeries->append("Failed", 1);
    m_missionSeries->append("Cancelled", 1);
    m_missionSeries->append("Rejected", 1);

    m_missionSeries->slices().at(0)->setColor(Qt::green);
    m_missionSeries->slices().at(1)->setColor(QColor(255, 165, 0));
    m_missionSeries->slices().at(2)->setColor(Qt::red);
    m_missionSeries->slices().at(3)->setColor(QColor(150, 150, 150));
    m_missionSeries->slices().at(4)->setColor(QColor(140, 80, 160));

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

    m_lblMissionStats = new QLabel("Background: 0/0 (—)\nInterrupt: 0");
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
    // Pie chart — one slice per scheduled mission terminal state.
    auto setSlice = [&](int idx, const char* label, int value) {
        m_missionSeries->slices().at(idx)->setValue(value);
        m_missionSeries->slices().at(idx)->setLabel(QString("%1: %2").arg(label).arg(value));
        m_missionSeries->slices().at(idx)->setLabelVisible(value > 0);
    };
    setSlice(0, "On Time",   msg->scheduled_on_time);
    setSlice(1, "Late",      msg->scheduled_late);
    setSlice(2, "Failed",    msg->scheduled_failed);
    setSlice(3, "Cancelled", msg->scheduled_cancelled);
    setSlice(4, "Rejected",  msg->scheduled_rejected);

    QString stateText = QString("Idle: %1\nMoving: %2\nSearching: %3\nAccompany: %4\nCharging: %5")
                            .arg(fmtTime(msg->idle_time))
                            .arg(fmtTime(msg->moving_time))
                            .arg(fmtTime(msg->searching_time))
                            .arg(fmtTime(msg->accompany_time))
                            .arg(fmtTime(msg->charging_time));
    m_lblRobotState->setText(stateText);

    const QString bgPercent = msg->background_total > 0
        ? QString::number(100.0 * msg->background_completed / msg->background_total, 'f', 0) + "%"
        : "—";
    QString missionText = QString("Background: %1/%2 (%3)\nInterrupt: %4")
                              .arg(msg->background_completed)
                              .arg(msg->background_total)
                              .arg(bgPercent)
                              .arg(msg->interrupt_total);
    m_lblMissionStats->setText(missionText);

    QString perfText = QString("Avg Early: %1s\nAvg Late: %2s")
                           .arg(QString::number(msg->avg_early_arrival, 'f', 1))
                           .arg(QString::number(msg->avg_lateness, 'f', 1));
    m_lblPerformance->setText(perfText);

    QString moveText = QString("Distance: %1m\nUtilization: %2%\nIdle: %3%\nCharging: %4%\nReturning: %5%")
                           .arg(QString::number(msg->total_distance, 'f', 2))
                           .arg(QString::number(msg->utilization, 'f', 1))
                           .arg(QString::number(msg->idle_percent, 'f', 1))
                           .arg(QString::number(msg->charging_percent, 'f', 1))
                           .arg(QString::number(msg->returning_percent, 'f', 1));
    m_lblMovement->setText(moveText);
}

}  // namespace des_metrics_panel

PLUGINLIB_EXPORT_CLASS(des_metrics_panel::DesMetricsPanel, rviz_common::Panel)
