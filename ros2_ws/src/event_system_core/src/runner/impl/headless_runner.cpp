#include "headless_runner.h"
#include <memory>
#include <queue>
#include "../../behaviour/bt_setup.h"
#include "event_system_msgs/srv/detail/set_system_state__struct.hpp"
#include "../../model/event_queue.h"

void HeadlessRunner::setupApplication(const std::string& path) {
    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Application...");

    assert(std::filesystem::exists(path) && std::filesystem::is_directory(path));

    m_config = std::make_shared<des::SimConfig>(ConfigLoader::loadSimConfig().value());
    const auto people = ConfigLoader::loadEmployees(CONFIG_PATH + "employee.json");
    assert(!people.value().empty());

    for (const auto& p: people.value()) {
        m_employeeLocations[p->firstName] = p->roomLabels;
    } 
    m_scheduler = std::make_unique<Scheduler>(m_config, m_plannerNode, m_employeeLocations);
    
    m_ctx = std::make_shared<SimulationContext>(
        m_eventQueue,
        m_config,
        m_plannerNode,
        m_employeeLocations,
        *m_scheduler
    );

    std::queue<std::string> files;
    for (const auto &entry : std::filesystem::directory_iterator(path)) {
        files.push(entry.path());
    }

    IAppRunner::scheduleOccupancy(*m_config, people.value(), m_ctx->m_rng);
    m_eventQueue.extend(IAppRunner::personArrivalGenerator(people.value(),  "5.2B_Elevator"));
    m_eventQueue.push(std::make_shared<ResetEvent>(0, files));

    m_ctx->addObserver(m_metricsNode);
    m_ctx->resetContext(m_eventQueue.top()->time);
    m_ctx->setBehaviorTree(setupBehaviorTree(m_ctx));

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Setup Complete!");
}

int HeadlessRunner::loadAppState() const {
    return event_system_msgs::srv::SetSystemState::Request::RUN;
}

void HeadlessRunner::reset() {
    // not implemented yet
}

void HeadlessRunner::enterPause() const {
    // not implemented yet
}

void HeadlessRunner::updateConfig() {
    // not implemented yet
}
