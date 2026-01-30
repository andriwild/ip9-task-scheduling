#pragma once

#include <qcheckbox.h>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "event_system_msgs/msg/system_config.hpp"
#include "event_system_msgs/srv/set_system_config.hpp"

using ServiceResponseFuture = rclcpp::Client<event_system_msgs::srv::SetSystemConfig>::SharedFuture;
namespace des_system_config {

class DesSystemConfig : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit DesSystemConfig(QWidget* parent = 0);
    ~DesSystemConfig() = default;
    void onInitialize() override;

protected:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> m_nodePtr;
    rclcpp::Client<event_system_msgs::srv::SetSystemConfig>::SharedPtr m_client;
    rclcpp::Subscription<event_system_msgs::msg::SystemConfig>::SharedPtr m_subscriber;

    QDoubleSpinBox* m_findPersonProb;
    QDoubleSpinBox* m_driveTimeStd;
    QDoubleSpinBox* m_robotSpeed;
    QDoubleSpinBox* m_robotAccompanySpeed;
    QDoubleSpinBox* m_conversationProbability;
    QDoubleSpinBox* m_conversationDurationStd;
    QDoubleSpinBox* m_timeBuffer;
    QCheckBox* m_cacheEnabled;
    QLineEdit* m_appointmentsPath;

    QPushButton* m_btnSetConfig;
    QLabel* m_statusLabel;

private Q_SLOTS:
    void onSetConfig();
    void onServiceResponse(ServiceResponseFuture future);
    void onSystemConfig(const event_system_msgs::msg::SystemConfig::SharedPtr msg);
};

}  // namespace des_system_config
