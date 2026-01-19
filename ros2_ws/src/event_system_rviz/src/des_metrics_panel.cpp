#include <QGroupBox>
#include <QVBoxLayout>
#include <event_system_rviz/des_metrics_panel.hpp>
#include <iomanip>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <sstream>

namespace des_metrics_panel {

DesMetricsPanel::DesMetricsPanel(QWidget * parent) : rviz_common::Panel(parent) {
    auto mainLayout = new QVBoxLayout(this);

    // 1. Mission Performance Chart (Donut)
    m_missionSeries = new QPieSeries();
    m_missionSeries->setHoleSize(0.35);
    // Initialize with 0
    m_missionSeries->append("On Time", 1);
    m_missionSeries->append("Late", 1);
    m_missionSeries->append("Failed", 1);

    // Set colors
    m_missionSeries->slices().at(0)->setColor(Qt::green);
    m_missionSeries->slices().at(0)->setLabelVisible(true);
    m_missionSeries->slices().at(1)->setColor(QColor(255, 165, 0));  // Orange
    m_missionSeries->slices().at(2)->setColor(Qt::red);

    auto chart = new QChart();
    chart->addSeries(m_missionSeries);
    chart->setTitle("Mission Performance");
    chart->legend()->setAlignment(Qt::AlignRight);

    m_chartView = new QChartView(chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);
    m_chartView->setMinimumHeight(250);
    mainLayout->addWidget(m_chartView);

    // 2. Stats Grid
    auto statsGroup = new QGroupBox("Statistics");
    auto grid = new QGridLayout(statsGroup);

    m_lblRobotState = new QLabel("Idle: 0s\nMoving: 0s\nSearching: 0s\nAccompany: 0s\nCharging: 0s");
    grid->addWidget(new QLabel("<b>Robot State:</b>"), 0, 0);
    grid->addWidget(m_lblRobotState, 1, 0);

    m_lblMissionStats = new QLabel("Total: 0\nOn Time: 0\nLate: 0\nFailed: 0\nCancelled: 0");
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

DesMetricsPanel::~DesMetricsPanel() = default;

void DesMetricsPanel::onInitialize() {
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = node_abstraction->get_raw_node();

    m_subscriber = m_node->create_subscription<event_system_msgs::msg::MetricsReport>(
        "/metrics_report", rclcpp::QoS(10),
        [this](const event_system_msgs::msg::MetricsReport::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                this->onMetricsReport(msg);
            });
        });
}

QString DesMetricsPanel::fmtTime(int s) {
    int m = s / 60;
    int sec = s % 60;
    std::ostringstream oss;
    oss << m << "m " << std::setw(2) << std::setfill('0') << sec << "s";
    return QString::fromStdString(oss.str());
}

void DesMetricsPanel::onMetricsReport(const event_system_msgs::msg::MetricsReport::SharedPtr msg) {
    // Update Chart
    // Clear and redo seems easiest to handle varying values, or update existing slices
    // Note: QPieSeries takes ownership of slices, but updating values is safer via slices() list
    // Indices: 0=OnTime, 1=Late, 2=Failed

    // Check if we have data to avoid empty chart logic or division by zero visual issues
    // Just update values directly
    m_missionSeries->slices().at(0)->setValue(msg->missions_on_time);
    m_missionSeries->slices().at(0)->setLabel(QString("On Time: %1").arg(msg->missions_on_time));

    m_missionSeries->slices().at(1)->setValue(msg->missions_late);
    m_missionSeries->slices().at(1)->setLabel(QString("Late: %1").arg(msg->missions_late));

    // Combine failed and cancelled for visual simplicity or add another slice?
    // User asked for "one time, late, failed". I'll treat cancelled as failed or ignore?
    // Let's add Cancelled for completeness or merge. User prompt: "Mission Performance (one time, late, failed) in einem donout chart"
    // So I will stick to 3 or maybe 4 slices if I want to be precise.
    // I put 3 in constructor. Let's merge Cancelled into Failed or just show Failed.
    // Let's just show Failed as per request.
    m_missionSeries->slices().at(2)->setValue(msg->missions_failed + msg->missions_cancelled);  // Merging for "Not Success" logic
    m_missionSeries->slices().at(2)->setLabel(QString("Failed: %1").arg(msg->missions_failed + msg->missions_cancelled));

    // Update Text Stats
    QString stateText = QString("Idle: %1\nMoving: %2\nSearching: %3\nAccompany: %4\nCharging: %5")
                            .arg(fmtTime(msg->idle_time))
                            .arg(fmtTime(msg->moving_time))
                            .arg(fmtTime(msg->searching_time))
                            .arg(fmtTime(msg->accompany_time))
                            .arg(fmtTime(msg->charging_time));
    m_lblRobotState->setText(stateText);

    QString missionText = QString("Total: %1\nOn Time: %2\nLate: %3\nFailed: %4\nCancelled: %5")
                              .arg(msg->total_missions)
                              .arg(msg->missions_on_time)
                              .arg(msg->missions_late)
                              .arg(msg->missions_failed)
                              .arg(msg->missions_cancelled);
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
