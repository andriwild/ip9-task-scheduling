#pragma once

#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/byte.hpp>

#include "event_system_interfaces/srv/set_des_state.hpp"

using ServiceResponseFuture = rclcpp::Client<event_system_interfaces::srv::SetDesState>::SharedFuture;
namespace des_panel {

class DesPanel : public rviz_common::Panel {
    Q_OBJECT

    public:
    explicit DesPanel(QWidget * parent = 0);
    ~DesPanel() override;
    void onInitialize() override;

protected:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
    rclcpp::Service<event_system_interfaces::srv::SetDesState>::SharedPtr service_;
    rclcpp::Client<event_system_interfaces::srv::SetDesState>::SharedPtr client;

    QLabel * m_stateLabel;
    QPushButton * m_btnRun;
    QPushButton * m_btnPause;
    QPushButton * m_btnReset;

private Q_SLOTS:
    void btnRunClick();
    void btnPauseClick();
    void btnResetClick();
    void onServiceResponse(ServiceResponseFuture future);
};

} 

