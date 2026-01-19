#pragma once

#include <qobjectdefs.h>

#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "event_system_msgs/msg/timeline_event.hpp"
#include "timeline.hpp"

namespace des_timeline_panel {

class DesTimelinePanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit DesTimelinePanel(QWidget * parent = nullptr);
    ~DesTimelinePanel() override;
    void onInitialize() override;

private:
    void onTimelineEvent(const event_system_msgs::msg::TimelineEvent::SharedPtr msg);

    Timeline * m_timeline;
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<event_system_msgs::msg::TimelineEvent>::SharedPtr m_subscriber;
    int m_minStartTime;
    int m_maxEndTime;
};

}  // namespace des_timeline_panel
