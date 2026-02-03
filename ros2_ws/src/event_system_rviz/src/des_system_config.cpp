#include <qcheckbox.h>

#include <QFormLayout>
#include <QHBoxLayout>
#include <event_system_rviz/des_system_config.hpp>
#include <memory>
#include <rviz_common/display_context.hpp>

namespace des_system_config {

DesSystemConfig::DesSystemConfig(QWidget * parent) : Panel(parent) {
    const auto layout = new QFormLayout(this);

    m_findPersonProb = new QDoubleSpinBox();
    m_findPersonProb->setRange(0.0, 1.0);
    m_findPersonProb->setSingleStep(0.01);

    m_driveTimeStd = new QDoubleSpinBox();
    m_driveTimeStd->setRange(0.0, 10.0);

    m_robotSpeed = new QDoubleSpinBox();
    m_robotSpeed->setRange(0.0, 10.0);

    m_robotAccompanySpeed = new QDoubleSpinBox();
    m_robotAccompanySpeed->setRange(0.0, 10.0);

    m_conversationProbability = new QDoubleSpinBox();
    m_conversationProbability->setRange(0.0, 1.0);

    m_conversationDurationStd = new QDoubleSpinBox();
    m_conversationDurationStd->setRange(0.0, 200.0);

    m_conversationDurationMean = new QDoubleSpinBox();
    m_conversationDurationMean->setRange(0.0, 200.0);

    m_timeBuffer = new QDoubleSpinBox();
    m_timeBuffer->setRange(0.0, 1000.0);

    m_energyConsumptionDrive = new QDoubleSpinBox();
    m_energyConsumptionDrive->setRange(0.0, 1000.0);

    m_energyConsumptionBase = new QDoubleSpinBox();
    m_energyConsumptionBase->setRange(0.0, 1000.0);

    m_batteryCapacity = new QDoubleSpinBox();
    m_batteryCapacity->setRange(0.0, 10000.0);

    m_chargingRate = new QDoubleSpinBox();
    m_chargingRate->setRange(0.0, 1000.0);

    m_lowBatteryThreshold = new QDoubleSpinBox();
    m_lowBatteryThreshold->setRange(0.0, 1.0);
    m_lowBatteryThreshold->setSingleStep(0.01);

    auto dockLayout = new QHBoxLayout();
    for (int i = 0; i < 3; ++i) {
        m_dockPose[i] = new QDoubleSpinBox();
        m_dockPose[i]->setRange(-1000.0, 1000.0);
        dockLayout->addWidget(m_dockPose[i]);
    }

    m_cacheEnabled = new QCheckBox();
    m_cacheEnabled->setEnabled(true);

    m_appointmentsPath = new QLineEdit();
    m_appointmentsPath->setText("appointments.json");

    m_btnSetConfig = new QPushButton("Set Config");
    m_statusLabel = new QLabel("[status]");

    layout->addRow("Find Person Prob:", m_findPersonProb);
    layout->addRow("Drive Time Std:", m_driveTimeStd);
    layout->addRow("Robot Speed:", m_robotSpeed);
    layout->addRow("Accompany Speed:", m_robotAccompanySpeed);
    layout->addRow("Conv Prob:", m_conversationProbability);
    layout->addRow("Conv Duration Std:", m_conversationDurationStd);
    layout->addRow("Conv Duration Mean:", m_conversationDurationMean);
    layout->addRow("Time Buffer:", m_timeBuffer);
    layout->addRow("Energy Cons Drive:", m_energyConsumptionDrive);
    layout->addRow("Energy Cons Base:", m_energyConsumptionBase);
    layout->addRow("Battery Capacity:", m_batteryCapacity);
    layout->addRow("Charging Rate:", m_chargingRate);
    layout->addRow("Low Battery Thresh:", m_lowBatteryThreshold);
    layout->addRow("Dock Pose (x,y,yaw):", dockLayout);
    layout->addRow("Cache Enabled:", m_cacheEnabled);
    layout->addRow("Appointments Path:", m_appointmentsPath);
    layout->addRow(m_btnSetConfig);
    layout->addRow(m_statusLabel);

    QObject::connect(m_btnSetConfig, &QPushButton::released, this, &DesSystemConfig::onSetConfig);
}

void DesSystemConfig::onInitialize() {
    rclcpp::Node::SharedPtr node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    m_client = node->create_client<event_system_msgs::srv::SetSystemConfig>("/set_des_config");
    m_subscriber = node->create_subscription<event_system_msgs::msg::SystemConfig>(
        "/system_config", rclcpp::QoS(1).transient_local(),
        [this](const event_system_msgs::msg::SystemConfig::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onSystemConfig(msg); });
        });
}

void DesSystemConfig::onSetConfig() {
    auto request = std::make_shared<event_system_msgs::srv::SetSystemConfig::Request>();

    request->find_person_probability = m_findPersonProb->value();
    request->drive_time_std = m_driveTimeStd->value();
    request->robot_speed = m_robotSpeed->value();
    request->robot_accompany_speed = m_robotAccompanySpeed->value();
    request->conversation_probability = m_conversationProbability->value();
    request->conversation_duration_std = m_conversationDurationStd->value();
    request->conversation_duration_mean = m_conversationDurationMean->value();
    request->time_buffer = m_timeBuffer->value();
    request->energy_consumption_drive = m_energyConsumptionDrive->value();
    request->energy_consumption_base = m_energyConsumptionBase->value();
    request->battery_capacity = m_batteryCapacity->value();
    request->charging_rate = m_chargingRate->value();
    request->low_battery_threshold = m_lowBatteryThreshold->value();
    request->dock_pose = {
        (float)m_dockPose[0]->value(),
        (float)m_dockPose[1]->value(),
        (float)m_dockPose[2]->value()};

    request->cache_enabled = m_cacheEnabled->isChecked();
    request->appointments_path = m_appointmentsPath->text().toStdString();

    m_statusLabel->setText("Sending...");

    m_client->async_send_request(request, std::bind(&DesSystemConfig::onServiceResponse, this, std::placeholders::_1));
}

void DesSystemConfig::onServiceResponse(ServiceResponseFuture future) {
    try {
        auto response = future.get();
        QMetaObject::invokeMethod(this, [this, response]() {
            if (response->success) {
                this->m_statusLabel->setText("Success: " + QString::fromStdString(response->message));
            } else {
                this->m_statusLabel->setText("Failed: " + QString::fromStdString(response->message));
            }
        });
    } catch (const std::exception& e) {
        QMetaObject::invokeMethod(
            this, [this, e]() { this->m_statusLabel->setText(QString("Error: %1").arg(e.what())); });
    }
}

void DesSystemConfig::onSystemConfig(const event_system_msgs::msg::SystemConfig::SharedPtr msg) {
    m_findPersonProb->setValue(msg->find_person_probability);
    m_driveTimeStd->setValue(msg->drive_time_std);
    m_robotSpeed->setValue(msg->robot_speed);
    m_robotAccompanySpeed->setValue(msg->robot_accompany_speed);
    m_conversationProbability->setValue(msg->conversation_probability);
    m_conversationDurationStd->setValue(msg->conversation_duration_std);
    m_conversationDurationMean->setValue(msg->conversation_duration_mean);
    m_timeBuffer->setValue(msg->time_buffer);
    m_energyConsumptionDrive->setValue(msg->energy_consumption_drive);
    m_energyConsumptionBase->setValue(msg->energy_consumption_base);
    m_batteryCapacity->setValue(msg->battery_capacity);
    m_chargingRate->setValue(msg->charging_rate);
    m_lowBatteryThreshold->setValue(msg->low_battery_threshold);

    if (msg->dock_pose.size() >= 3) {
        m_dockPose[0]->setValue(msg->dock_pose[0]);
        m_dockPose[1]->setValue(msg->dock_pose[1]);
        m_dockPose[2]->setValue(msg->dock_pose[2]);
    }

    m_cacheEnabled->setChecked(msg->cache_enabled);
    m_appointmentsPath->setText(QString::fromStdString(msg->appointments_path));
}

}  // namespace des_system_config

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(des_system_config::DesSystemConfig, rviz_common::Panel)
