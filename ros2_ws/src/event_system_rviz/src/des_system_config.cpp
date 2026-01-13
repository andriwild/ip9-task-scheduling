#include <QFormLayout>
#include <event_system_rviz/des_system_config.hpp>
#include <memory>
#include <rviz_common/display_context.hpp>

namespace des_system_config {

DesSystemConfig::DesSystemConfig(QWidget * parent) : Panel(parent) {
    const auto layout = new QFormLayout(this);

    m_findPersonProb = new QDoubleSpinBox();
    m_findPersonProb->setRange(0.0, 1.0);
    m_findPersonProb->setSingleStep(0.01);
    m_findPersonProb->setValue(0.5);

    m_driveStd = new QDoubleSpinBox();
    m_driveStd->setRange(0.0, 100.0);
    m_driveStd->setValue(1.0);

    m_robotSpeed = new QDoubleSpinBox();
    m_robotSpeed->setRange(0.0, 10.0);
    m_robotSpeed->setValue(1.0);

    m_robotEscortSpeed = new QDoubleSpinBox();
    m_robotEscortSpeed->setRange(0.0, 10.0);
    m_robotEscortSpeed->setValue(0.5);

    m_conversationFoundStd = new QDoubleSpinBox();
    m_conversationFoundStd->setRange(0.0, 100.0);
    m_conversationFoundStd->setValue(5.0);

    m_conversationDropOffStd = new QDoubleSpinBox();
    m_conversationDropOffStd->setRange(0.0, 100.0);
    m_conversationDropOffStd->setValue(5.0);

    m_missionOverhead = new QDoubleSpinBox();
    m_missionOverhead->setRange(0.0, 100.0);
    m_missionOverhead->setValue(10.0);

    m_timeBuffer = new QDoubleSpinBox();
    m_timeBuffer->setRange(0.0, 100.0);
    m_timeBuffer->setValue(5.0);

    m_enableLogging = new QCheckBox();

    m_appointmentsPath = new QLineEdit();
    m_appointmentsPath->setText("appointments.json");

    m_btnSetConfig = new QPushButton("Set Config");
    m_statusLabel = new QLabel("[status]");

    layout->addRow("Find Person Prob:", m_findPersonProb);
    layout->addRow("Drive Std:", m_driveStd);
    layout->addRow("Robot Speed:", m_robotSpeed);
    layout->addRow("Escort Speed:", m_robotEscortSpeed);
    layout->addRow("Conv Found Std:", m_conversationFoundStd);
    layout->addRow("Conv DropOff Std:", m_conversationDropOffStd);
    layout->addRow("Mission Overhead:", m_missionOverhead);
    layout->addRow("Time Buffer:", m_timeBuffer);
    layout->addRow("Enable Logging:", m_enableLogging);
    layout->addRow("Appointments Path:", m_appointmentsPath);
    layout->addRow(m_btnSetConfig);
    layout->addRow(m_statusLabel);

    QObject::connect(m_btnSetConfig, &QPushButton::released, this, &DesSystemConfig::onSetConfig);
}

DesSystemConfig::~DesSystemConfig() = default;

void DesSystemConfig::onInitialize() {
    m_nodePtr = getDisplayContext()->getRosNodeAbstraction().lock();
    rclcpp::Node::SharedPtr node = m_nodePtr->get_raw_node();
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
    request->drive_std = m_driveStd->value();
    request->robot_speed = m_robotSpeed->value();
    request->robot_escort_speed = m_robotEscortSpeed->value();
    request->conversation_found_std = m_conversationFoundStd->value();
    request->conversation_drop_off_std = m_conversationDropOffStd->value();
    request->mission_overhead = m_missionOverhead->value();
    request->time_buffer = m_timeBuffer->value();
    request->enable_logging = m_enableLogging->isChecked();
    request->appointments_path = m_appointmentsPath->text().toStdString();

    m_statusLabel->setText("Sending...");

    m_client->async_send_request(
        request, std::bind(&DesSystemConfig::onServiceResponse, this, std::placeholders::_1));
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
    m_driveStd->setValue(msg->drive_std);
    m_robotSpeed->setValue(msg->robot_speed);
    m_robotEscortSpeed->setValue(msg->robot_escort_speed);
    m_conversationFoundStd->setValue(msg->conversation_found_std);
    m_conversationDropOffStd->setValue(msg->conversation_drop_off_std);
    m_missionOverhead->setValue(msg->mission_overhead);
    m_timeBuffer->setValue(msg->time_buffer);
    m_enableLogging->setChecked(msg->enable_logging);
    m_appointmentsPath->setText(QString::fromStdString(msg->appointments_path));
}

}  // namespace des_system_config

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(des_system_config::DesSystemConfig, rviz_common::Panel)
