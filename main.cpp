#include <QApplication>
#include <QtConcurrent/QtConcurrent>
// #include <behaviortree_cpp/blackboard.h>
// #include "behaviour/nodes/robot_state.h"

#include <cassert>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <iostream>
#include <ostream>
#include <memory>


#include <gz/transport.hh>
#include <gz/msgs.hh>

#include "util/types.h"
#include "model/robot.h"
#include "model/event.h"
#include "model/robot.h"
#include "model/context.h"
#include "view/term.h"
#include "sim/ros/path_node.h"
#include "sim/ros/marker.h"
#include "sim/gz_lib.h"
#include "rclcpp/rclcpp.hpp"

/*
 * Proces: (exogene / endogen events)
 *
 * Bootstraping phase:
 * backward scheduling (meetings)
 * Machbarkeitsprüfung (konflikte bei der plaunung)
 *
 * Simulation phase:
 * Laufzeitkonflikte
 *
 *
 */

const int SIM_START_TIME = 8 * 3600; 
const int NODE_LOBBY = 0; // The therapy location
const double ROBOT_SPEED = 1.0; // units per second

std::mt19937 rng(42); // Fixed seed for reproducibility
//


// Calculates travel time for SIMULATION (Includes noise/randomness)
// double getRealTravelTime(int nodeA, int nodeB) {
//     double baseTime = estimateTravelTime(nodeA, nodeB);
//     // Add random delay (0 to 10% extra time)
//     std::uniform_real_distribution<double> dist(1.0, 1.1); 
//     return baseTime * dist(rng);
// }
std::map<std::string, std::vector<std::string>> employeeLocations = {
    {"Max", {"5.2B03", "5.2B03", "Kitchen", "Open Zone"}},
    {"Leo", {"5.2B15", "5.2B10", "5.2B33", "Kitchen"}},
    {"Fred", {"Open Zone", "Kitchen"}}
};

des::Appointment a1 = {1, "Max",  "Printer", SIM_START_TIME + 2*3600, "Massage"};
des::Appointment a2 = {2, "Leo",  "Printer", SIM_START_TIME + 5*3600, "Nachhilfe"};
des::Appointment a3 = {3, "Fred", "Printer", SIM_START_TIME + 8*3600, "Mitarbeiter Gespräch"};

std::map<std::string, des::Point> locationMap = {
    // Gazebo coordinates
    {"0/0", {0.0, 0.0}},

    // imvs left side
    {"5.2B01", {8.0,  -17.0}},
    {"5.2B03", {2.2,  -14.0}},
    {"5.2B04", {-0.8, -12.4}},
    {"5.2B05", {-4.0, -10.6}},

    // imvs front side
    {"5.2B010", {-4.8, -4.0}},
    {"5.2B013", {-3.3, 1.75}},
    {"5.2B015", {-2.0, 7.6}},
    {"5.2B016", {-0.3, 14.3}},
    {"5.2B018", {0.7,  19.0}},

    // imvs meeting rooms 
    {"5.2B31", {1.2, -8.2}},
    {"5.2B33", {1.2, -4.8}},
    {"5.2B34", {1.2, -1.3}},

    // Special Locations
    {"Dock",   {-2.75, -4.66}},
    {"Printer",   {3.0,   6.0}},
    {"Open Zone", {-7.6,  -9.2}},
    {"Kitchen", {8.0,  -11.6}},
    {"Floor",     {-1.6,  -7.8}}
};

EventQueue eventQueue;

double currentSimTime = SIM_START_TIME;

Robot robot;

des::Point mapToRosLocation(des::Point& point) {
    return {point.m_y, -point.m_x};
}



std::optional<double> estimateTravelTime(
    std::shared_ptr<PathPlannerNode> planner,
    const double speed, 
    const std::string& from, 
    const std::string& to) 
{
    auto fromPoint = mapToRosLocation(locationMap[from]);
    auto toPoint   = mapToRosLocation(locationMap[to]);
    auto fromPose  = SimplePose(fromPoint.m_x, fromPoint.m_y, 0.0);
    auto toPose    = SimplePose(toPoint.m_x, toPoint.m_y, 0.0);

    auto result = planner->computeDistance(fromPose, toPose);

    if (result.success) {
        std::cout << "Distance: " << result.distance << " meters" << std::endl;
        return result.distance / ROBOT_SPEED;
    } else {
        std::cerr << "Failed to calculate path: " << from << " -> " << to << std::endl;
    }
    return std::nullopt;
}

int main(int argc, char *argv[]) {
    std::cout << "\n--- Descrete Event Sytem ---\n\n";

    rclcpp::init(argc, argv);
    auto marker_node = std::make_shared<MarkerPublisher>();

    std::vector<MapLocation> rosLocations = {};
    for (auto [name, p] : locationMap) {
        auto point = mapToRosLocation(p);
        rosLocations.emplace_back(name, point.m_x, point.m_y);
    }
    marker_node->publishLocations(rosLocations);

    auto planner_node = std::make_shared<PathPlannerNode>();
    std::thread ros_thread;

    if(planner_node->isReady()) {
        std::cout << "Planner ready!" << std::endl;

        ros_thread = std::thread([planner_node] { 
            rclcpp::spin(planner_node); 
        });
    } else {
        std::cerr << "Planner initialization failed!" << std::endl;
        return 1;
    }


    // auto p1 = mapToRosLocation(locationMap["Printer"]);
    // auto p2 = mapToRosLocation(locationMap["5.2B01"]);
    // SimplePose start{p1.m_x, p1.m_y, 0.0};
    // SimplePose goal{p2.m_x, p2.m_y, 0.0};
    // PathResult result = planner_node->computeDistance(start, goal);

    
    // if (result.success) {
    //     std::cout << "Distance: " << result.distance << " meters" << std::endl;
    // } else {
    //     std::cout << "Failed to calculate path" << std::endl;
    // }


    SimulationContext ctx(robot, eventQueue, planner_node);

    TerminalView termView;
    ctx.addObserver(std::make_shared<TerminalView>(termView));

    std::vector<des::Appointment> dailyPlan = { a1, a2, a3 };

    eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));

    for (const auto& appt : dailyPlan) {
        auto employeeLocation = employeeLocations[appt.personName].front();
        std::string startPos = "Dock";
        std::optional<double> travelTo = estimateTravelTime(planner_node, des::ROBOT_SPEED, startPos, employeeLocation);
        std::optional<double> escorting = estimateTravelTime(planner_node, des::ROBOT_SPEED, employeeLocation, appt.roomName);
        std::optional<double> travelBack = estimateTravelTime(planner_node, des::ROBOT_SPEED, appt.roomName, startPos);
        double taskOverhead = 30.0 + 120.0; // Scan + Interaction + Search 
        double buffer = 3 * 60.0; // 3 min safety buffer 
        
        std::cout << "value check" << std::endl;
        assert(travelTo.has_value());
        assert(escorting.has_value());
        assert(travelBack.has_value());

        std::cout << "calc start" << std::endl;
        double startSeconds = appt.appointmentTime - (travelTo.value() + escorting.value() + travelBack.value() + taskOverhead + buffer);
        std::cout << "enqueuing" << std::endl;
        std::cout << startSeconds<< std::endl;

        eventQueue.push(std::make_shared<MissionDispatchEvent>(static_cast<int>(startSeconds)));
        std::cout << "done" << std::endl;
    }

    std::cout << "run loop" << std::endl;
    while (!eventQueue.empty()) {
        std::cout << "1" << std::endl;
        auto e = eventQueue.top();
        eventQueue.pop();
        std::cout << "2" << std::endl;
        ctx.setTime(e->time);
        std::cout << "3" << std::endl;
        e->execute(ctx);
    }

    //QApplication app(argc, argv);
    // DBClient db_client(argv[1], argv[2]);
    // auto points = db_client.entrances();
    //
    // if (points.has_value()) {
    //     for (const auto& p: points.value()) {
    //         std::cout << p << std::endl;
    //         sim::addMarker(std::to_string(p.m_x), -p.m_y, p.m_x, 2.8);
    //     }
    // }
    // auto p = db_client.personByName("Andri", "Wild");
    // std::cout << p->id << p->firstName << std::endl;
    //
    // // for (const auto& [name, p]: points) {
    // //     sim::addMarker(name, p.m_x, p.m_y, 2.8);
    // // }
    //
    // BT::BehaviorTreeFactory factory;
    // factory.registerNodeType<DriveStart>("DriveStart");
    // factory.registerNodeType<NavigateToPoint>("NavigateToPoint", &robot, &graph, &roClock);
    // factory.registerNodeType<EventHandler>("EventHandler", &eventTree, &roClock);
    // factory.registerNodeType<RobotState>("RobotState", &robot);
    // factory.registerNodeType<SayHello>("SayHello");
    // factory.registerNodeType<LogNode>("LoggerNode");
    //
    // m_bt = factory.createTreeFromFile("../tree.xml");
    // m_bt.rootBlackboard().get()->set("goal", -1);

    // BT::StdCoutLogger logger(m_bt);

    // Timeline timelineView(model, maxSimTime);
    // timelineView.show();

    //BT::printTreeRecursively(tree.rootNode());
    //
    // std::string xml_models = BT::writeTreeNodesModelXML(factory);
    // std::cout << xml_models << std::endl;

    // QtConcurrent::run([&] {
    //     for (int i = 0; i < maxSimTime; ++i) {
    //         model.simStep();
    //         m_bt.sleep(std::chrono::milliseconds(100));
    //     }
    // });

    //QApplication::exec();
    
    std::cin.get();
    rclcpp::shutdown();
    if(ros_thread.joinable()){
        ros_thread.join();
    }
    return 0;
}
