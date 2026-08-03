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

#ifndef DES_CONFIG_DIR
#define DES_CONFIG_DIR "config"
#endif

const QString DEFAULT_CONFIG_FILE_LOCATION = qEnvironmentVariableIsSet("DES_CONFIG_DIR")
    ? qEnvironmentVariable("DES_CONFIG_DIR")
    : QString(DES_CONFIG_DIR);

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
    QDoubleSpinBox* m_timeBuffer;
    QDoubleSpinBox* m_energyConsumptionDrive;
    QDoubleSpinBox* m_energyConsumptionIdle;
    QDoubleSpinBox* m_batteryCapacity;
    QDoubleSpinBox* m_initialBatteryCapacity;
    QDoubleSpinBox* m_chargingRate;
    QDoubleSpinBox* m_lowBatteryThreshold;
    QDoubleSpinBox* m_fullBatteryThreshold;
    QDoubleSpinBox* m_batteryVoltage;
    QDoubleSpinBox* m_cvThreshold;
    QDoubleSpinBox* m_taperFraction;
    QCheckBox* m_chargeToFull;
    QCheckBox* m_alwaysChargeAtDock;
    QTimeEdit* m_arrivalMean;
    QDoubleSpinBox* m_arrivalStd;
    QTimeEdit* m_departureMean;
    QDoubleSpinBox* m_departureStd;
    QComboBox* m_arrivalDistribution;
    QComboBox* m_departureDistribution;
    QTimeEdit* m_simStartTime;
    QDoubleSpinBox* m_simDuration;
    QLineEdit* m_dockLocation;
    QLineEdit* m_peopleSpawnLocation;
    QDoubleSpinBox* m_personDetectionRange;
    QCheckBox* m_cacheEnabled;
    QLabel* m_appointmentsPath;
    QPushButton* m_btnSetConfig;
    QLabel* m_statusLabel;
    QPushButton* m_btnFileDialog;
    QString m_configFile;

private Q_SLOTS:
    void onSetConfig();
    void onServiceResponse(ServiceResponseFuture future);
    void onSystemConfig(event_system_msgs::msg::SystemConfig::SharedPtr msg);
    QTreeWidgetItem* addConfigItem(QTreeWidgetItem* parent, QString label, QWidget* widget);
};

}  // namespace des_system_config
