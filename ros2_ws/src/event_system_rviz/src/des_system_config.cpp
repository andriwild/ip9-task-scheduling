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

    m_driveTimeStd = new QDoubleSpinBox();
    m_driveTimeStd->setRange(0.0, 10.0);

    m_robotSpeed = new QDoubleSpinBox();
    m_robotSpeed->setRange(0.0, 10.0);

    m_robotAccompanySpeed = new QDoubleSpinBox();
    m_robotAccompanySpeed->setRange(0.0, 10.0);

    m_conversationProbability = new QDoubleSpinBox();
    m_conversationProbability->setRange(0.0, 1.0);

    m_conversationDurationStd= new QDoubleSpinBox();
    m_conversationDurationStd->setRange(0.0, 1.0);

    m_timeBuffer = new QDoubleSpinBox();
    m_timeBuffer->setRange(0.0, 1000.0);

    m_appointmentsPath = new QLineEdit();
    m_appointmentsPath->setText("appointments.json");

    m_btnSetConfig = new QPushButton("Set Config");
    m_statusLabel = new QLabel("[status]");

    layout->addRow("Find Person Prob:" , m_findPersonProb);
    layout->addRow("Drive Time Std:"   , m_driveTimeStd);
    layout->addRow("Robot Speed:"      , m_robotSpeed);
    layout->addRow("Accompany Speed:"  , m_robotAccompanySpeed);
    layout->addRow("Conv Prob:"        , m_conversationProbability);
    layout->addRow("Conv Duration Std:" , m_conversationDurationStd);
    layout->addRow("Time Buffer:"      , m_timeBuffer);
    layout->addRow("Appointments Path:", m_appointmentsPath);
    layout->addRow(m_btnSetConfig);
    layout->addRow(m_statusLabel);

    QObject::connect(m_btnSetConfig, &QPushButton::released, this, &DesSystemConfig::onSetConfig);
}


void DesSystemConfig::onInitialize() {
    rclcpp::Node::SharedPtr node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    m_client     = node->create_client<event_system_msgs::srv::SetSystemConfig>("/set_des_config");
    m_subscriber = node->create_subscription<event_system_msgs::msg::SystemConfig>(
        "/system_config", rclcpp::QoS(1).transient_local(),
        [this](const event_system_msgs::msg::SystemConfig::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onSystemConfig(msg); });
        });
}

void DesSystemConfig::onSetConfig() {
    auto request = std::make_shared<event_system_msgs::srv::SetSystemConfig::Request>();

    request->find_person_probability   = m_findPersonProb->value();
    request->drive_time_std            = m_driveTimeStd->value();
    request->robot_speed               = m_robotSpeed->value();
    request->robot_accompany_speed     = m_robotAccompanySpeed->value();
    request->conversation_probability  = m_conversationProbability->value();
    request->conversation_duration_std = m_conversationDurationStd->value();
    request->time_buffer               = m_timeBuffer->value();
    request->appointments_path         = m_appointmentsPath->text().toStdString();

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
    m_findPersonProb          ->setValue(msg->find_person_probability);
    m_driveTimeStd            ->setValue(msg->drive_time_std);
    m_robotSpeed              ->setValue(msg->robot_speed);
    m_robotAccompanySpeed     ->setValue(msg->robot_accompany_speed);
    m_timeBuffer              ->setValue(msg->time_buffer);
    m_conversationProbability ->setValue(msg->conversation_probability);
    m_conversationDurationStd ->setValue(msg->conversation_duration_std);
    m_appointmentsPath        ->setText(QString::fromStdString(msg->appointments_path));
}

}  // namespace des_system_config

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(des_system_config::DesSystemConfig, rviz_common::Panel)
