#include <QApplication>
#include <QtConcurrent/QtConcurrent>
// #include <behaviortree_cpp/blackboard.h>
// #include "behaviour/nodes/robot_state.h"

#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <iostream>
#include <ostream>
#include <memory>


#include "util/types.h"
#include "model/robot.h"
#include "model/event.h"
#include "model/robot.h"
#include "model/context.h"
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

const double SIM_START_TIME = 8.0 * 3600.0; 
const int NODE_LOBBY = 0; // The therapy location
const double ROBOT_SPEED = 1.0; // units per second

std::mt19937 rng(42); // Fixed seed for reproducibility
//
double getDistance(int nodeA, int nodeB) {
    return std::abs(nodeA - nodeB) * 10.0;
}

// Calculates travel time for PLANNING (Deterministic)
double estimateTravelTime(int nodeA, int nodeB) {
    double dist = getDistance(nodeA, nodeB);
    return dist / ROBOT_SPEED; 
}

// Calculates travel time for SIMULATION (Includes noise/randomness)
double getRealTravelTime(int nodeA, int nodeB) {
    double baseTime = estimateTravelTime(nodeA, nodeB);
    // Add random delay (0 to 10% extra time)
    std::uniform_real_distribution<double> dist(1.0, 1.1); 
    return baseTime * dist(rng);
}



des::Appointment a1 = {1, 101, 2, SIM_START_TIME + 2*3600, "Patient A (Room 2, 10:00)"};
des::Appointment a2 = {2, 102, 5, SIM_START_TIME + 5*3600, "Patient B (Room 5, 13:00)"};
des::Appointment a3 = {3, 103, 8, SIM_START_TIME + 8*3600, "Patient C (Room 8, 16:00)"};

EventQueue eventQueue;
std::map<int, des::Appointment> schedule;

double currentSimTime = SIM_START_TIME;

Robot robot;


int main(int argc, char *argv[]) {
    SimulationContext ctx(robot, eventQueue);
    std::cout << "\nRun Descrete Event Sytem ...\n" << std::endl;

    schedule[1] = a1;
    schedule[3] = a3;
    schedule[2] = a2;


    std::vector<des::Appointment> dailyPlan = {a1, a2, a3};

    eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));
    std::cout << "--- BOOTSTRAPPING SCHEDULE ---" << std::endl;

    for (const auto& appt : dailyPlan) {
        double travelTo = estimateTravelTime(NODE_LOBBY, appt.roomNodeId);
        double travelBack = estimateTravelTime(appt.roomNodeId, NODE_LOBBY) * 1.5; // Escort is slower
        double taskOverhead = 30.0 + 120.0; // Scan + Interaction
        double buffer = 600.0; // 10 min safety buffer

        double startSeconds = appt.appointmentTime - (travelTo + travelBack + taskOverhead + buffer);

        eventQueue.push(std::make_shared<MissionDispatchEvent>(startSeconds));
        std::cout << "Planned Dispatch for " << appt.description 
                  << " at " << toHumanReadable(startSeconds) << std::endl;
    }

    while (!eventQueue.empty()) {
        auto e = eventQueue.top();
        eventQueue.pop();
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
    //
    // rclcpp::init(argc, argv);
    // // auto node = std::make_shared<GazeboTfBroadcaster>();
    // auto planner_node = std::make_shared<PathPlannerNode>();
    // // std::thread ros_thread([node] { rclcpp::spin(node); });
    // // std::thread ros_thread2([planner_node] { rclcpp::spin(planner_node); });
    // //
    // while (!planner_node->isReady()) {
    //     std::cout << "Waiting for planner..." << std::endl;
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    // std::cout << "Planner ready!" << std::endl;

    //
    //
    // std::cout << "start calc test\n";
    // for (int i = 0; i < 10; i ++) {
    //
    //     SimplePose start{0.5, 0.5, 0.0};
    //     SimplePose goal{1.5, static_cast<double>(i), 0.0};
    //     PathResult result = planner_node->computeDistance(start, goal);
    //     std::cout << "end calc test\n";
    //
    //     if (result.success) {
    //         std::cout << "Distance: " << result.distance << " meters" << std::endl;
    //     } else {
    //         std::cout << "Failed" << std::endl;
    //     }
    // }

    // EV::Tree<IEvent> eventTree;
    // //
    // BT::Tree m_bt;
    // SimTime m_simTime;
    // // ReadOnlyClock roClock(&m_simTime);
    // //
    // //
    // Robot robot(0, 0);
    // Simulation model(&robot, eventTree, m_bt, &m_simTime);

    // EV::Tree<SimulationEvent> dockTree;
    // auto dockTreeRoot = dockTree.createRoot(std::make_unique<Tour>(100, "Tour"));
    // planner.tour(dockTreeRoot, robotPosition, 10);
    // root->addSubtree(dockTree.releaseRoot());

    // std::cout << eventTree << std::endl;
    //
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
    //rclcpp::shutdown();
    // ros_thread.join();
    // ros_thread2.join();
    return 0;
}
