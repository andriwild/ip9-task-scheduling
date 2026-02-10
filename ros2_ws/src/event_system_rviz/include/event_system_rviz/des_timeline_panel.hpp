#pragma once

#include <qobjectdefs.h>

#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "event_system_msgs/msg/timeline_event.hpp"
#include "event_system_msgs/msg/timeline_state_change.hpp"
#include "event_system_msgs/msg/timeline_meeting.hpp"
#include "event_system_msgs/msg/timeline_reset.hpp"

#include "event_system_rviz/timeline/timeline.hpp"

namespace des_timeline_panel {

class DesTimelinePanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit DesTimelinePanel(QWidget * parent = nullptr);
    ~DesTimelinePanel() override;
    void onInitialize() override;

private:
    void onStateChange(const event_system_msgs::msg::TimelineStateChange::SharedPtr msg);
    void onMeeting(const event_system_msgs::msg::TimelineMeeting::SharedPtr msg);
    void onReset(const event_system_msgs::msg::TimelineReset::SharedPtr msg);
    void onEvent(const event_system_msgs::msg::TimelineEvent::SharedPtr msg);

    Timeline* m_timeline;
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<event_system_msgs::msg::TimelineStateChange>::SharedPtr m_subStateChange;
    rclcpp::Subscription<event_system_msgs::msg::TimelineMeeting>::SharedPtr m_subMeeting;
    rclcpp::Subscription<event_system_msgs::msg::TimelineReset>::SharedPtr m_subReset;
    rclcpp::Subscription<event_system_msgs::msg::TimelineEvent>::SharedPtr m_subEvent;

    int m_minStartTime;
    int m_maxEndTime;
};

}  // namespace des_timeline_panel
