#include "engine/context.h"
#include "util/log.h"

#include <utility>
#include "util/geometry.h"
#include "util/rnd.h"
#include "sim/scheduler.h"

SimulationContext::SimulationContext(
    EventQueue& queue,
    std::shared_ptr<des::SimConfig> simConfig,
    std::shared_ptr<IPathPlanner> plannerNode,
    des::PersonList people,
    des::RoomMap rooms
)
    : m_simConfig(std::move(simConfig))
    , m_queue(queue)
    , m_scheduler(std::make_unique<Scheduler>(m_simConfig, plannerNode, m_rooms))
    , m_plannerNode(std::move(plannerNode))
    , m_rooms(std::move(rooms))
    , m_persons(std::move(people), m_rooms, m_simConfig->seed + des::PLACEMENT_SEED_OFFSET)
    , m_missions(m_queue, m_eventBus)
{
    reseed(m_simConfig->seed);
    m_robot = std::make_unique<Robot>(m_simConfig, m_currentTime);
    DES_LOG_DEBUG(rclcpp::get_logger("des.context"), "Simulation Context created!");
}

void SimulationContext::reseed(const unsigned int seed) {
    m_activeSeed = seed;
    m_worldRng.seed(seed);
    m_robotRng.seed(seed + des::ROBOT_SEED_OFFSET);
    m_persons.reseed(seed + des::PLACEMENT_SEED_OFFSET);
    DES_LOG_DEBUG(rclcpp::get_logger("des.context"), "RNG seeded with %u", seed);
}

void SimulationContext::reseedPersons() {
    m_persons.reseedPersons(m_activeSeed + des::PERSON_SEED_BASE);
}

unsigned int SimulationContext::activeSeed() const {
    return m_activeSeed;
}

Journey SimulationContext::scheduleArrival(const std::string& target) const {
    const std::optional<double> distance = this->m_plannerNode->calcDistance(
        this->m_robot->getLocation(), 
        target,
        m_simConfig->cacheEnabled
    );
    assert(distance.has_value());

    const double travelTime = distance.value() / this->m_robot->getCurrentSpeed();
    double travelTimeRnd = rnd::normal(m_robotRng, travelTime, getDriveTimeStd());
    if (travelTimeRnd < 0) {
        travelTimeRnd = travelTime;
    }
    return { travelTimeRnd, distance.value() };
}

void SimulationContext::completeOrder(const des::OrderPtr& order) {
    m_missions.complete(*this, order);
}

void SimulationContext::resetContext(const int newTime) {
    m_currentTime = newTime;
    des::log::setSimTime(newTime);
    m_missions.reset();
    m_persons.clearRooms();
    m_services.clear();
    resetRobot();
}

des::OrderPtr SimulationContext::peekNextScheduledOrder() const {
    return m_missions.peekNextScheduledOrder();
}

std::vector<des::OrderPtr> SimulationContext::peekScheduledOrdersUntil(const int untilTime) const {
    return m_missions.peekScheduledOrdersUntil(untilTime);
}

void SimulationContext::setConfig(const std::shared_ptr<des::SimConfig> &newConfig) {
    m_simConfig = newConfig;
    m_robot->updateConfig(*newConfig);
    DES_LOG_DEBUG(rclcpp::get_logger("des.context"), "Config updated");
}

void SimulationContext::changeRobotState(std::unique_ptr<RobotState> newState) const {
    const auto* current = m_robot->getState();
    const auto newType = newState->getType();
    const bool sameState = current
        && current->getType() == newType
        && current->getName() == newState->getName();
    const bool leavingOpportunisticCharge = current
        && current->getType() == des::RobotStateType::CHARGING
        && newType != des::RobotStateType::CHARGING
        && m_robot->m_opportunisticCharge;
    m_robot->changeState(std::move(newState), m_currentTime);
    if (leavingOpportunisticCharge) {
        m_queue.cancelByType(des::EventType::BATTERY_FULL);
        m_queue.cancelByType(des::EventType::CHARGE_PHASE_TRANSITION);
        m_robot->m_batteryFullEventScheduled = false;
        m_robot->m_opportunisticCharge = false;
    }
    if (!sameState) {
        notifyRobotStateChanged();
    }
}

bool SimulationContext::pushInterrupt(const des::OrderPtr& order) {
    return m_missions.pushInterrupt(*this, order);
}

void SimulationContext::popInterrupt(const des::OrderPtr& completedOrder) {
    m_missions.popInterrupt(*this, completedOrder);
}

bool SimulationContext::hasActiveInterrupt() const {
    return m_missions.hasActiveInterrupt();
}

Scheduler& SimulationContext::getScheduler() {
    return *m_scheduler;
}

const Scheduler& SimulationContext::getScheduler() const {
    return *m_scheduler;
}

void SimulationContext::resetRobot() {
    m_robot.reset();
    m_robot = std::make_unique<Robot>(m_simConfig, m_currentTime);
}

std::optional<double> SimulationContext::getDistance(const std::string& from, const std::string& to) const {
    return m_plannerNode->calcDistance(from, to, m_simConfig->cacheEnabled);
}

int SimulationContext::getTime() const {
    return m_currentTime;
}

std::shared_ptr<des::SimConfig> SimulationContext::getConfig() const {
    return m_simConfig;
}

Robot* SimulationContext::getRobot() const {
    return m_robot.get();
}

std::mt19937& SimulationContext::worldRng() const {
    return m_worldRng;
}

std::mt19937& SimulationContext::robotRng() const {
    return m_robotRng;
}

void SimulationContext::pushEvent(const std::shared_ptr<IEvent>& event) {
    m_queue.push(event);
}

void SimulationContext::startActivity(const std::shared_ptr<IEvent>& endEvent) {
    m_robot->setInFlight(endEvent);
    m_queue.push(endEvent);
}

void SimulationContext::executeEvent(const std::shared_ptr<IEvent>& event) {
    m_currentEvent = event.get();
    event->execute(*this);
    m_currentEvent = nullptr;
    if (m_robot->inFlight().lock() == event) {
        m_robot->clearInFlight();
    }
}

void SimulationContext::setBehaviorTree(std::shared_ptr<BT::Tree> tree) {
    m_behaviorTree = std::move(tree);
}

void SimulationContext::tickBT() {
    const auto inFlight = m_robot->inFlight().lock();
    const bool ownCompletionTick = !inFlight || inFlight.get() == m_currentEvent;
    const bool interruptActive = m_missions.hasActiveInterrupt();

    if (!ownCompletionTick && !interruptActive) {
        return;
    }
    m_behaviorTree->tickOnce();
}

void SimulationContext::setBTBlackboard(const std::string& key, const std::string& value) {
    m_behaviorTree->rootBlackboard()->set(key, value);
}

des::Person* SimulationContext::getPersonByName(const std::string& person) const {
    return m_persons.getByName(person);
}

const des::PersonList& SimulationContext::getAllPersons() const {
    return m_persons.all();
}

bool SimulationContext::hasEmployee(const std::string& person) const {
    return m_persons.hasEmployee(person);
}

std::string SimulationContext::getPersonLocation(const std::string& name) const {
    return m_persons.room(name);
}

const std::map<std::string, std::string>& SimulationContext::getAllPersonLocations() const {
    return m_persons.allRooms();
}

void SimulationContext::setPersonLocation(const std::string& name, const std::string& room) {
    m_persons.setRoom(name, room);
}

std::optional<des::Point> SimulationContext::getPersonPosition(const std::string& name) const {
    return m_persons.position(name);
}

bool SimulationContext::robotSeesPerson(const std::string& name) const {
    if (!m_persons.isAt(name, m_robot->getLocation())) {
        return false;
    }
    const auto pos = m_persons.position(name);
    if (!pos) {
        return false;
    }
    const des::Point robotPos = m_robot->getPosition();
    if (std::hypot(pos->m_x - robotPos.m_x, pos->m_y - robotPos.m_y) > m_simConfig->personDetectionRange) {
        return false;
    }
    return geom::isVisible(*pos, m_robot->getVisibility());
}

std::optional<int> SimulationContext::lastServiced(const std::string& room, const std::string& type) const {
    return m_services.lastServiced(room, type);
}

void SimulationContext::recordServiced(const std::string& room, const std::string& type, const int time) {
    m_services.recordServiced(room, type, time);
}

std::vector<std::string> SimulationContext::roomNames() const {
    std::vector<std::string> names;
    names.reserve(m_rooms.size());
    for (const auto& [name, loc] : m_rooms) {
        names.push_back(name);
    }
    return names;
}

const des::Room& SimulationContext::room(const std::string& name) const {
    static const des::Room unknown{"", des::Point{}};
    const auto it = m_rooms.find(name);
    if (it == m_rooms.end()) {
        DES_LOG_DEBUG(rclcpp::get_logger("des.context"), "Room '%s' not found", name.c_str());
        return unknown;
    }
    return it->second;
}

void SimulationContext::setOrderPtr(const des::OrderPtr& orderPtr) {
    m_missions.setCurrent(orderPtr);
}

des::OrderPtr SimulationContext::getOrderPtr() const {
    return m_missions.effective();
}

void SimulationContext::updateOrderState(const des::MissionState& newState) {
    m_missions.updateState(newState);
}

void SimulationContext::addScheduledMission(const des::OrderPtr orderPtr) {
    m_missions.addScheduled(orderPtr);
}

bool SimulationContext::hasScheduledMission() const {
    return m_missions.hasScheduled();
}

des::OrderPtr SimulationContext::nextScheduledMission() {
    return m_missions.nextScheduled();
}

des::OrderPtr SimulationContext::popScheduledMission() {
    return m_missions.popScheduled();
}

void SimulationContext::addBackgroundMission(const des::OrderPtr orderPtr) {
    m_missions.addBackground(orderPtr);
}

bool SimulationContext::hasBackgroundMission() const {
    return m_missions.hasBackground();
}

des::OrderPtr SimulationContext::acceptFeasibleBackgroundMission() {
    return m_missions.acceptFeasibleBackground(*this);
}

std::optional<int> SimulationContext::getNextScheduledDispatchTime() const {
    return m_missions.nextScheduledDispatchTime();
}

std::optional<int> SimulationContext::getSimulationEndTime() const {
    return m_queue.nextEventTime(des::EventType::SIMULATION_END);
}

void SimulationContext::publishMission(const des::OrderPtr& order, const int time) {
    m_eventBus.notifyMissionPublished(order, time);
}


void SimulationContext::advanceTime(const int newTime) {
    assert(newTime >= m_currentTime);
    m_robot->updateBatteryBalance(newTime, m_robot->getState()->getEnergyConsumption(*m_robot, *m_simConfig));
    if (m_robot->isBatteryLow()) {
        m_robot->setChargingRequired(true);
    }
    m_currentTime = newTime;
    des::log::setSimTime(newTime);
}

void SimulationContext::addObserver(const std::shared_ptr<IObserver>& observer) {
    m_eventBus.addObserver(observer);
}

void SimulationContext::removeObserver(const std::shared_ptr<IObserver>& observer) {
    m_eventBus.removeObserver(observer);
}


void SimulationContext::notifyRobotStateChanged() const {
    const auto* state = m_robot->getState();
    m_eventBus.notifyStateChanged(m_currentTime, state->getType(), state->getName(), m_robot->batteryStats());
}

void SimulationContext::notifyBatteryChanged() const {
    notifyRobotStateChanged();
}

void SimulationContext::notifyEvent(const IEvent& event) const {
    int missionId = event.getMissionId();
    if (missionId < 0) {
        if (const auto current = m_missions.current()) {
            missionId = current->id;
        }
    }
    m_eventBus.notifyEvent(event.time, event.getType(), event.getName(), m_robot->isDriving(), m_robot->isCharging(), event.getColor(), missionId);
}

void SimulationContext::notifyChargeStarted() const {
    m_robot->beginChargeSession(m_currentTime);
    m_eventBus.notifyEvent(m_currentTime, des::EventType::CHARGE_MISSION_START, "Start Charging", m_robot->isDriving(), m_robot->isCharging(), "", -1);
}

void SimulationContext::robotMoved(const std::string& location, const double distance) const {
    m_robot->setLocation(location);
    m_robot->addDistance(distance);
    const auto it = m_rooms.find(location);
    if (it != m_rooms.end()) {
        m_robot->setPosition(it->second.m_waypoint);
    }
}

void SimulationContext::robotMovedTo(const des::Point& position, const double distance) const {
    m_robot->setPosition(position);
    m_robot->addDistance(distance);
}

double SimulationContext::getDriveTimeStd() const {
    return m_simConfig->driveTimeStd;
}
