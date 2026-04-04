#pragma once

#include <qcheckbox.h>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimeEdit>
#include <QTreeWidget>
#include <QFileDialog>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "event_system_msgs/msg/system_config.hpp"
#include "event_system_msgs/srv/set_system_config.hpp"

const QString DEFAULT_CONFIG_FILE_LOCATION = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config";

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
    QDoubleSpinBox* m_fullBatteryThreshold;
    QTimeEdit* m_arrivalMean;
    QDoubleSpinBox* m_arrivalStd;
    QTimeEdit* m_departureMean;
    QDoubleSpinBox* m_departureStd;
    QComboBox* m_arrivalDistribution;
    QComboBox* m_departureDistribution;
    QDoubleSpinBox* m_appointmentDuration;
    QLineEdit* m_dockLocation;
    QCheckBox* m_cacheEnabled;
    QLabel* m_appointmentsPath;
    QLabel* m_employeePath;
    QPushButton* m_btnSetConfig;
    QLabel* m_statusLabel;
    QPushButton* m_fileDialogBtn;
    QPushButton* m_btnFileDialog;
    QPushButton* m_btnEmployeeFileDialog;
    QFileDialog* m_fileDialog;
    QString m_configFile;
    QString m_employeeFile;

private Q_SLOTS:
    void onSetConfig();
    void onServiceResponse(ServiceResponseFuture future);
    void onSystemConfig(event_system_msgs::msg::SystemConfig::SharedPtr msg);
    QString onChooseFile();
    QTreeWidgetItem* addConfigItem(QTreeWidgetItem* parent, QString label, QWidget* widget);
};

}  // namespace des_system_config
