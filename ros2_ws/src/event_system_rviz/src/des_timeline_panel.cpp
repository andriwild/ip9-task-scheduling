#include <event_system_rviz/des_timeline_panel.hpp>
#include <limits>
#include <memory>
#include <rviz_common/display_context.hpp>

namespace des_timeline_panel {

DesTimelinePanel::DesTimelinePanel(QWidget * parent) : Panel(parent) {
    auto layout = new QVBoxLayout(this);

    // Hardcoded start/end time for now, ideally this should be configurable or received from core
    constexpr int HOUR = 3600;
    constexpr int SIM_START_TIME = 8 * HOUR;
    constexpr int SIM_END_TIME = SIM_START_TIME + 12 * HOUR;

    m_timeline = new Timeline(SIM_START_TIME, SIM_END_TIME);
    layout->addWidget(m_timeline);
    m_minStartTime = std::numeric_limits<int>::max();
    m_maxEndTime = std::numeric_limits<int>::min();
}

DesTimelinePanel::~DesTimelinePanel() = default;

void DesTimelinePanel::onInitialize() {
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = node_abstraction->get_raw_node();

    m_subscriber = m_node->create_subscription<event_system_msgs::msg::TimelineEvent>(
        "/timeline_events", rclcpp::QoS(100),
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

        m_minStartTime = std::min(m_minStartTime, msg->time);
        // The appointment time we reconstructed seems to represent the END time?
        // Wait, looking at the code: appt->appointmentTime = msg->time + msg->duration;
        // In the core (ros.cpp): msg.time = appt->appointmentTime; (START time)
        // msg.duration = appt->appointmentTime - startTime; (Wait, this logic in core seems odd?)

        // Let's re-read ros.cpp snippet from earlier step 23:
        // msg.time = appt->appointmentTime;  // Using startTime as the event time
        // ...
        // msg.duration = appt->appointmentTime - startTime;

        // Ok, assuming msg->time is START time and msg->duration is duration.
        // appt->appointmentTime usually means the start time of the appointment.
        // But in the subscriber code I saw: `appt->appointmentTime = msg->time + msg->duration;`
        // This looks like it might be calculating the END time?
        // Let's assume standard definitions: msg->time is start, msg->duration is length.
        // So end time is msg->time + msg->duration.

        int currentEndTime = msg->time + msg->duration;
        m_maxEndTime = std::max(m_maxEndTime, currentEndTime);

        // Update range: [Min - 1h, Max + 1h]
        int newStart = m_minStartTime - 3600;
        int newEnd = m_maxEndTime + 3600;
        m_timeline->setRange(newStart, newEnd);

        // Wait, the subscriber code I'm replacing had:
        // appt->appointmentTime = msg->time + msg->duration;
        // If I change the logic around `appt` I might break how it's drawn if `drawMeetingPlan` expects something specific.
        // `Timeline::drawMeetingPlan` uses `appt.get()->appointmentTime`.
        // Let's check `Timeline::drawMeetingPlan` in `timeline.hpp` (Step 40).
        // `double meetingX = timeToX(appt.get()->appointmentTime);`
        // It draws a rect from `startTime` (passed as arg, which is `msg->time`) to `meetingX`.
        // So `appt->appointmentTime` is indeed treated as the END time in `Timeline` class?
        // Let's check `ros.cpp` again.
        // `msg.time = appt->appointmentTime`. Here it seems `appointmentTime` is START time.
        // This is confusing naming in the codebase, but I must preserve behavior.
        // If `Timeline::drawMeetingPlan` draws from `startTime` to `appt->appointmentTime`,
        // then `appt->appointmentTime` MUST be the end time for the rect to have processing width.
        // So `msg->time + msg->duration` is correct for the END time.

        m_timeline->addMeetingPlan(appt, msg->time);
    } else if (msg->type == event_system_msgs::msg::TimelineEvent::RESET) {
        m_minStartTime = std::numeric_limits<int>::max();
        m_maxEndTime = std::numeric_limits<int>::min();
        m_timeline->handleReset();
    }
}

}  // namespace des_timeline_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(des_timeline_panel::DesTimelinePanel, rviz_common::Panel)
