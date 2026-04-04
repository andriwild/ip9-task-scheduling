#include <QVBoxLayout>
#include <event_system_rviz/des_panel.hpp>
#include <memory>
#include <rviz_common/display_context.hpp>

#include "event_system_msgs/srv/set_system_state.hpp"

namespace des_panel {

DesPanel::DesPanel(QWidget *parent) : Panel(parent) {
    const auto layout = new QVBoxLayout(this);

    m_stateLabel = new QLabel("[no data]");

    m_btnRun   = new QPushButton("Run");
    m_btnPause = new QPushButton("Pause");
    m_btnReset = new QPushButton("Reset");
    m_btnStep  = new QPushButton("Step");

    layout->addWidget(m_stateLabel);

    layout->addWidget(m_btnRun);
    layout->addWidget(m_btnPause);
    layout->addWidget(m_btnReset);
    layout->addWidget(m_btnStep);

    QObject::connect(m_btnRun,   &QPushButton::released, this, &DesPanel::btnRunClick);
    QObject::connect(m_btnPause, &QPushButton::released, this, &DesPanel::btnPauseClick);
    QObject::connect(m_btnReset, &QPushButton::released, this, &DesPanel::btnResetClick);
    QObject::connect(m_btnStep,  &QPushButton::released, this, &DesPanel::btnStepClick);
}

DesPanel::~DesPanel() = default;

void DesPanel::onInitialize() {
    m_nodePtr = getDisplayContext()->getRosNodeAbstraction().lock();

    rclcpp::Node::SharedPtr node = m_nodePtr->get_raw_node();

    m_client = node->create_client<event_system_msgs::srv::SetSystemState>("set_des_state");
}

void DesPanel::btnRunClick() {
    auto request = std::make_shared<event_system_msgs::srv::SetSystemState_Request>();
    request->command_id = event_system_msgs::srv::SetSystemState::Request::RUN;

    m_client->async_send_request(
        request,
        std::bind(&DesPanel::onServiceResponse, this, std::placeholders::_1)
    );
}

void DesPanel::btnPauseClick() {
    auto request = std::make_shared<event_system_msgs::srv::SetSystemState_Request>();
    request->command_id = event_system_msgs::srv::SetSystemState::Request::PAUSE;

    m_client->async_send_request(
        request,
        std::bind(&DesPanel::onServiceResponse, this, std::placeholders::_1)
    );
}

void DesPanel::btnResetClick() {
    auto request = std::make_shared<event_system_msgs::srv::SetSystemState_Request>();
    request->command_id = event_system_msgs::srv::SetSystemState::Request::RESET;

    m_client->async_send_request(
        request,
        std::bind(&DesPanel::onServiceResponse, this, std::placeholders::_1)
    );
}


void DesPanel::btnStepClick() {
    auto request        = std::make_shared<event_system_msgs::srv::SetSystemState_Request>();
    request->command_id = event_system_msgs::srv::SetSystemState::Request::STEP;

    m_client->async_send_request(
        request,
        std::bind(&DesPanel::onServiceResponse, this, std::placeholders::_1)
    );
}

void DesPanel::onServiceResponse(ServiceResponseFuture future) {
    auto response = future.get();
    QMetaObject::invokeMethod(this, [this, response](){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %s", response->message.c_str());
        this->m_stateLabel->setText(QString::fromStdString(response->message));
    });
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(des_panel::DesPanel, rviz_common::Panel)
