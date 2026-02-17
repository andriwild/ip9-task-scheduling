#pragma once

#include <qcheckbox.h>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTreeWidget>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "event_system_msgs/msg/system_config.hpp"
#include "event_system_msgs/srv/set_system_config.hpp"

using ServiceResponseFuture = rclcpp::Client<event_system_msgs::srv::SetSystemConfig>::SharedFuture;
namespace des_system_config {

class DesSystemConfig final : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit DesSystemConfig(QWidget * parent = nullptr);
    ~DesSystemConfig() override = default;
    void onInitialize() override;

protected:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> m_nodePtr;
    rclcpp::Client<event_system_msgs::srv::SetSystemConfig>::SharedPtr m_client;
    rclcpp::Subscription<event_system_msgs::msg::SystemConfig>::SharedPtr m_subscriber;

    QTreeWidget* m_treeWidget;
    QDoubleSpinBox* m_findPersonProb;
    QDoubleSpinBox* m_driveTimeStd;
    QDoubleSpinBox* m_robotSpeed;
    QDoubleSpinBox* m_robotAccompanySpeed;
    QDoubleSpinBox* m_conversationProbability;
    QDoubleSpinBox* m_conversationDurationStd;
    QDoubleSpinBox* m_conversationDurationMean;
    QDoubleSpinBox* m_timeBuffer;
    QDoubleSpinBox* m_energyConsumptionDrive;
    QDoubleSpinBox* m_energyConsumptionIdle;
    QDoubleSpinBox* m_batteryCapacity;
    QDoubleSpinBox* m_initialBatteryCapacity;
    QDoubleSpinBox* m_chargingRate;
    QDoubleSpinBox* m_lowBatteryThreshold;
    QDoubleSpinBox* m_dockPose[3];
    QCheckBox* m_cacheEnabled;
    QLineEdit* m_appointmentsPath;
    QPushButton* m_btnSetConfig;
    QLabel* m_statusLabel;

private Q_SLOTS:
    void onSetConfig();
    void onServiceResponse(ServiceResponseFuture future);
    void onSystemConfig(const event_system_msgs::msg::SystemConfig::SharedPtr msg);
    QTreeWidgetItem* addConfigItem(QTreeWidgetItem* parent, QString label, QWidget* widget);
};

}  // namespace des_system_config
