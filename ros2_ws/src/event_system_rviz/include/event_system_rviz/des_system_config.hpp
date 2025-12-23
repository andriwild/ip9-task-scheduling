#pragma once

#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "event_system_msgs/srv/set_system_config.hpp"

using ServiceResponseFuture = rclcpp::Client<event_system_msgs::srv::SetSystemConfig>::SharedFuture;
namespace des_system_config {

class DesSystemConfig: public rviz_common::Panel {
    Q_OBJECT

public:
    explicit DesSystemConfig(QWidget * parent = 0);
    ~DesSystemConfig() override;
    void onInitialize() override;

protected:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> m_nodePtr;
    rclcpp::Client<event_system_msgs::srv::SetSystemConfig>::SharedPtr m_client;

    QDoubleSpinBox * m_findPersonProb;
    QDoubleSpinBox * m_driveStd;
    QDoubleSpinBox * m_robotSpeed;
    QDoubleSpinBox * m_robotEscortSpeed;
    QDoubleSpinBox * m_conversationFoundStd;
    QDoubleSpinBox * m_conversationDropOffStd;
    QDoubleSpinBox * m_missionOverhead;
    QDoubleSpinBox * m_timeBuffer;
    QCheckBox * m_enableLogging;

    QPushButton * m_btnSetConfig;
    QLabel * m_statusLabel;

private Q_SLOTS:
    void onSetConfig();
    void onServiceResponse(ServiceResponseFuture future);
};

}
