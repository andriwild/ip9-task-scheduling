#include <QVBoxLayout>
#include <event_system_rviz/des_panel.hpp>
#include <memory>
#include <rviz_common/display_context.hpp>

#include "event_system_interfaces/srv/set_des_state.hpp"

namespace des_panel {

enum SimState { IDLE = 0, RUN = 1 << 0, PAUSE = 1 << 1, RESET = 1 << 2 };


DesPanel::DesPanel(QWidget *parent) : Panel(parent) {
    const auto layout = new QVBoxLayout(this);

    m_stateLabel = new QLabel("[no data]");

    m_btnRun   = new QPushButton("Run");
    m_btnPause = new QPushButton("Pause");
    m_btnReset = new QPushButton("Reset");

    layout->addWidget(m_stateLabel);

    layout->addWidget(m_btnRun);
    layout->addWidget(m_btnPause);
    layout->addWidget(m_btnReset);

    QObject::connect(m_btnRun,   &QPushButton::released, this, &DesPanel::btnRunClick);
    QObject::connect(m_btnPause, &QPushButton::released, this, &DesPanel::btnPauseClick);
    QObject::connect(m_btnReset, &QPushButton::released, this, &DesPanel::btnResetClick);
}

DesPanel::~DesPanel() = default;

void DesPanel::onInitialize() {
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

    client = node->create_client<event_system_interfaces::srv::SetDesState>("set_des_state");
}

void DesPanel::btnRunClick() {
    auto request = std::make_shared<event_system_interfaces::srv::SetDesState_Request>();
    request->new_state = RUN;

    client->async_send_request(
        request,
        std::bind(&DesPanel::onServiceResponse, this, std::placeholders::_1)
    );
}

void DesPanel::btnPauseClick() {
    auto request = std::make_shared<event_system_interfaces::srv::SetDesState_Request>();
    request->new_state = PAUSE;

    client->async_send_request(
        request,
        std::bind(&DesPanel::onServiceResponse, this, std::placeholders::_1)
    );
}

void DesPanel::btnResetClick() {
    auto request = std::make_shared<event_system_interfaces::srv::SetDesState_Request>();
    request->new_state = RESET;

    client->async_send_request(
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
