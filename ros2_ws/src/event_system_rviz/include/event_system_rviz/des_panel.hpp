#pragma once

#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/byte.hpp>

#include "event_system_msgs/srv/set_system_state.hpp"

using ServiceResponseFuture = rclcpp::Client<event_system_msgs::srv::SetSystemState>::SharedFuture;
namespace des_panel {

class DesPanel : public rviz_common::Panel {
    Q_OBJECT

    public:
    explicit DesPanel(QWidget * parent = 0);
    ~DesPanel() override;
    void onInitialize() override;

protected:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> m_nodePtr;
    rclcpp::Service<event_system_msgs::srv::SetSystemState>::SharedPtr m_service;
    rclcpp::Client<event_system_msgs::srv::SetSystemState>::SharedPtr m_client;

    QLabel* m_stateLabel;
    QPushButton* m_btnRun;
    QPushButton* m_btnPause;
    QPushButton* m_btnReset;

private Q_SLOTS:
    void btnRunClick();
    void btnPauseClick();
    void btnResetClick();
    void onServiceResponse(ServiceResponseFuture future);
};

} 
