#include <qcheckbox.h>

#include <QFormLayout>
#include <QHBoxLayout>
#include <QTreeWidget>
#include <event_system_rviz/des_system_config.hpp>
#include <memory>
#include <rviz_common/display_context.hpp>

namespace des_system_config {

QTreeWidgetItem* DesSystemConfig::addConfigItem(QTreeWidgetItem* parent, QString label, QWidget* widget) {
    QTreeWidgetItem* item = new QTreeWidgetItem(parent);
    item->setText(0, label);
    m_treeWidget->setItemWidget(item, 1, widget);
    return item;
}

DesSystemConfig::DesSystemConfig(QWidget* parent) : Panel(parent) {
    const auto mainLayout = new QVBoxLayout(this);

    m_treeWidget = new QTreeWidget();
    m_treeWidget->setColumnCount(2);
    m_treeWidget->setHeaderLabels({"Parameter", "Value"});
    m_treeWidget->setIndentation(15);
    m_treeWidget->setAlternatingRowColors(true);

    // Movement
    QTreeWidgetItem* moveGroup = new QTreeWidgetItem(m_treeWidget);
    moveGroup->setText(0, "Movement");
    moveGroup->setExpanded(true);

    m_robotSpeed = new QDoubleSpinBox(); m_robotSpeed->setRange(0.0, 10.0);
    m_robotSpeed->setSuffix(" m/s");
    addConfigItem(moveGroup, "Robot Speed", m_robotSpeed);

    m_robotAccompanySpeed = new QDoubleSpinBox(); m_robotAccompanySpeed->setRange(0.0, 10.0);
    m_robotAccompanySpeed->setSuffix(" m/s");
    addConfigItem(moveGroup, "Accompany Speed", m_robotAccompanySpeed);

    m_driveTimeStd = new QDoubleSpinBox(); m_driveTimeStd->setRange(0.0, 10.0);
    addConfigItem(moveGroup, "Drive Time Std", m_driveTimeStd);

    // Interaction
    QTreeWidgetItem* interactionGroup = new QTreeWidgetItem(m_treeWidget);
    interactionGroup->setText(0, "Interaction");

    m_findPersonProb = new QDoubleSpinBox(); m_findPersonProb->setRange(0.0, 1.0); m_findPersonProb->setSingleStep(0.01);
    addConfigItem(interactionGroup, "Find Person Prob", m_findPersonProb);

    m_conversationProbability = new QDoubleSpinBox(); m_conversationProbability->setRange(0.0, 1.0);
    addConfigItem(interactionGroup, "Conv Prob", m_conversationProbability);

    m_conversationDurationMean = new QDoubleSpinBox(); m_conversationDurationMean->setRange(0.0, 200.0);
    m_conversationDurationMean->setSuffix(" s");
    addConfigItem(interactionGroup, "Conv Duration Mean", m_conversationDurationMean);

    m_conversationDurationStd = new QDoubleSpinBox(); m_conversationDurationStd->setRange(0.0, 200.0);
    addConfigItem(interactionGroup, "Conv Duration Std", m_conversationDurationStd);

    // Energy & Docking
    QTreeWidgetItem* energyGroup = new QTreeWidgetItem(m_treeWidget);
    energyGroup->setText(0, "Energy & Battery");

    m_batteryCapacity = new QDoubleSpinBox(); m_batteryCapacity->setRange(0.0, 100.0);
    m_batteryCapacity->setSuffix(" Ah");
    addConfigItem(energyGroup, "Battery Design Capacity", m_batteryCapacity);

    m_initialBatteryCapacity = new QDoubleSpinBox(); m_initialBatteryCapacity->setRange(0.0, m_batteryCapacity->maximum());
    m_initialBatteryCapacity->setSuffix(" Ah");
    addConfigItem(energyGroup, "Initial Battery Capacity", m_initialBatteryCapacity);

    m_chargingRate = new QDoubleSpinBox(); m_chargingRate->setRange(0.0, 5000.0);
    m_chargingRate->setSuffix(" W");
    addConfigItem(energyGroup, "Charging Power", m_chargingRate);

    m_lowBatteryThreshold = new QDoubleSpinBox(); m_lowBatteryThreshold->setRange(0.0, 100);
    m_lowBatteryThreshold->setSuffix(" %");
    addConfigItem(energyGroup, "Low Battery Threshold", m_lowBatteryThreshold);

    m_energyConsumptionDrive = new QDoubleSpinBox(); m_energyConsumptionDrive->setRange(0.0, 1000.0);
    m_energyConsumptionDrive->setSuffix(" W");
    addConfigItem(energyGroup, "Drive Power Load", m_energyConsumptionDrive);

    m_energyConsumptionIdle = new QDoubleSpinBox(); m_energyConsumptionIdle->setRange(0.0, 1000.0);
    m_energyConsumptionIdle->setSuffix(" W");
    addConfigItem(energyGroup, "Standby Power Load", m_energyConsumptionIdle);


    // General
    QTreeWidgetItem* generalGroup = new QTreeWidgetItem(m_treeWidget);
    generalGroup->setText(0, "General Settings");

    m_dockLocation= new QLineEdit("IMVS_Dock");
    addConfigItem(generalGroup, "Dock", m_dockLocation);

    m_timeBuffer = new QDoubleSpinBox(); m_timeBuffer->setRange(0.0, 1000.0);
    m_timeBuffer->setSuffix(" s");
    addConfigItem(generalGroup, "Time Buffer", m_timeBuffer);

    m_cacheEnabled = new QCheckBox();
    addConfigItem(generalGroup, "Cache Enabled", m_cacheEnabled);

    m_appointmentsPath = new QLineEdit("appointments.json");
    addConfigItem(generalGroup, "Appointments Path", m_appointmentsPath);

    // Untere Controls
    m_btnSetConfig = new QPushButton("Set Config");
    m_statusLabel = new QLabel("[status]");

    mainLayout->addWidget(m_treeWidget);
    mainLayout->addWidget(m_btnSetConfig);
    mainLayout->addWidget(m_statusLabel);

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
    request->energy_consumption_base = m_energyConsumptionIdle->value();
    request->battery_capacity = m_batteryCapacity->value();
    request->initial_battery_capacity = m_initialBatteryCapacity->value();
    request->charging_rate = m_chargingRate->value();
    request->low_battery_threshold = m_lowBatteryThreshold->value();
    request->dock_location = m_dockLocation->text().toStdString();
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
    m_energyConsumptionIdle->setValue(msg->energy_consumption_base);
    m_batteryCapacity->setValue(msg->battery_capacity);
    m_initialBatteryCapacity->setValue(msg->initial_battery_capacity);
    m_chargingRate->setValue(msg->charging_rate);
    m_lowBatteryThreshold->setValue(msg->low_battery_threshold);
    m_dockLocation->setText(QString::fromStdString(msg->dock_location));
    m_cacheEnabled->setChecked(msg->cache_enabled);
    m_appointmentsPath->setText(QString::fromStdString(msg->appointments_path));
}

}  // namespace des_system_config

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(des_system_config::DesSystemConfig, rviz_common::Panel)
