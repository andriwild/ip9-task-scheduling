#include <event_system_rviz/des_timeline_panel.hpp>
#include <limits>
#include <memory>
#include <rviz_common/display_context.hpp>
#include "event_system_msgs/msg/timeline_event.hpp"


constexpr int ONE_HOUR = 3600;

namespace des_timeline_panel {
using TimelineEvent = event_system_msgs::msg::TimelineEvent;

DesTimelinePanel::DesTimelinePanel(QWidget * parent) : Panel(parent) {
    auto layout = new QVBoxLayout(this);
    m_timeline  = new Timeline();

    layout->addWidget(m_timeline);
    m_minStartTime = std::numeric_limits<int>::max();
    m_maxEndTime   = std::numeric_limits<int>::min();
}

DesTimelinePanel::~DesTimelinePanel() = default;

void DesTimelinePanel::onInitialize() {
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = node_abstraction->get_raw_node();

    m_subscriber = m_node->create_subscription<TimelineEvent>(
        "/timeline_events", rclcpp::QoS(100),
        [this](const TimelineEvent::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                this->onTimelineEvent(msg);
            });
        });
}

void DesTimelinePanel::onTimelineEvent(const TimelineEvent::SharedPtr msg) {
    switch(msg->type) {
        case TimelineEvent::LOG:
            m_timeline->handleLog(msg->appointment_time, QString::fromStdString(msg->label));
            break;

        case TimelineEvent::MOVE:
            m_timeline->handleMove(msg->appointment_time, QString::fromStdString(msg->label));
            break;

        case TimelineEvent::STATE_CHANGE:
            m_timeline->handleStateChange(msg->appointment_time, msg->state);
            break;

        case TimelineEvent::MEETING: {
            auto appt = std::make_shared<des::Appointment>();
            appt->appointmentTime = msg->appointment_time;
            appt->description     = msg->description;
            appt->personName      = msg->person_name;

            m_minStartTime = std::min(m_minStartTime, msg->robot_start_time);
            m_maxEndTime   = std::max(m_maxEndTime  , appt->appointmentTime);

            m_timeline->setRange(m_minStartTime - ONE_HOUR, m_maxEndTime + ONE_HOUR);
            m_timeline->addMeetingPlan(appt, msg->robot_start_time);
            break;
        }

        case TimelineEvent::RESET: {
            m_minStartTime = std::numeric_limits<int>::max();
            m_maxEndTime   = std::numeric_limits<int>::min();
            m_timeline->handleReset();
            break;
        }
        default: 
            RCLCPP_WARN(m_node->get_logger(), "Timeline Event not handled");
    }
}

}  // namespace des_timeline_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(des_timeline_panel::DesTimelinePanel, rviz_common::Panel)
