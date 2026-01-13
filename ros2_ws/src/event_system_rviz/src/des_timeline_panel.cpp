#include <event_system_rviz/des_timeline_panel.hpp>
#include <rviz_common/display_context.hpp>
#include <memory>

namespace des_timeline_panel {

DesTimelinePanel::DesTimelinePanel(QWidget * parent) : Panel(parent) {
    auto layout = new QVBoxLayout(this);

    // Hardcoded start/end time for now, ideally this should be configurable or received from core
    constexpr int HOUR = 3600;
    constexpr int SIM_START_TIME = 8 * HOUR;
    constexpr int SIM_END_TIME = SIM_START_TIME + 12 * HOUR;

    m_timeline = new Timeline(SIM_START_TIME, SIM_END_TIME);
    layout->addWidget(m_timeline);
}

DesTimelinePanel::~DesTimelinePanel() = default;

void DesTimelinePanel::onInitialize() {
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = node_abstraction->get_raw_node();

    m_subscriber = m_node->create_subscription<event_system_msgs::msg::TimelineEvent>(
        "/timeline_events", rclcpp::QoS(10),
        [this](const event_system_msgs::msg::TimelineEvent::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                this->onTimelineEvent(msg);
            });
        });
}

void DesTimelinePanel::onTimelineEvent(const event_system_msgs::msg::TimelineEvent::SharedPtr msg) {
    if (msg->type == event_system_msgs::msg::TimelineEvent::LOG) {
        m_timeline->handleLog(msg->time, QString::fromStdString(msg->label));
    } else if (msg->type == event_system_msgs::msg::TimelineEvent::MOVE) {
        m_timeline->handleMove(msg->time, QString::fromStdString(msg->label));
    } else if (msg->type == event_system_msgs::msg::TimelineEvent::STATE_CHANGE) {
        m_timeline->handleStateChange(msg->time, msg->state);
    } else if (msg->type == event_system_msgs::msg::TimelineEvent::MEETING) {
        auto appt = std::make_shared<des::Appointment>();
        appt->description = msg->description;
        appt->personName = msg->person_name;
        appt->appointmentTime = msg->time + msg->duration;  // Reconstruct appointment time

        m_timeline->addMeetingPlan(appt, msg->time);
    } else if (msg->type == event_system_msgs::msg::TimelineEvent::RESET) {
        m_timeline->handleReset();
    }
}

}  // namespace des_timeline_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(des_timeline_panel::DesTimelinePanel, rviz_common::Panel)
