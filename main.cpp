#include <QApplication>
#include <QtConcurrent/QtConcurrent>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>

#include <cassert>
#include <iostream>
#include <ostream>
#include <memory>

#include "util/types.h"
#include "util/data.h"

#include "model/robot.h"
#include "model/event.h"
#include "model/robot.h"
#include "model/context.h"
#include "model/robot_state.h"

#include "view/term.h"
#include "view/gz.h"

#include "sim/ros/path_node.h"
#include "sim/ros/marker.h"

#include "behaviour/nodes/sim.h"

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

des::Appointment a1 = {1, "Max",  "Printer", SIM_START_TIME + 2*3600, "Massage"};
des::Appointment a2 = {2, "Leo",  "Printer", SIM_START_TIME + 5*3600, "Nachhilfe"};
des::Appointment a3 = {3, "Fred", "Printer", SIM_START_TIME + 8*3600, "Mitarbeiter Gespräch"};


void printQueue(EventQueue queue) {
    int size = queue.size();
    for(int i = 0; i< size; ++i){
        std::cout << *queue.top() << std::endl;
        queue.pop();
    }
}

int main(int argc, char *argv[]) {
    std::cout << "\n--- Descrete Event Sytem ---\n\n";


    rclcpp::init(argc, argv);
    auto marker_node = std::make_shared<MarkerPublisher>();

    std::vector<MapLocation> rosLocations = {};
    for (auto [name, p] : locationMap) {
        rosLocations.emplace_back(name, p.m_x, p.m_y);
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

    EventQueue eventQueue;
    Robot robot;

    auto tte = std::make_shared<TravelTimeEstimator>(planner_node, locationMap);
    SimulationContext ctx(robot, eventQueue, tte, employeeLocations);

    ctx.addObserver(std::make_shared<TerminalView>(TerminalView()));
    ctx.addObserver(std::make_shared<GazeboView>(GazeboView(locationMap)));

    std::vector<des::Appointment> dailyPlan = { a1, a2, a3 };

    eventQueue.push(std::make_shared<SimulationStartEvent>(SIM_START_TIME));

    for (const auto& appt : dailyPlan) {
        auto employeeLocation = employeeLocations[appt.personName].front();
        std::string startPos = robot.getIdleLocation();
        std::optional<double> travelTo   = tte->estimateDuration(startPos, employeeLocation, DEFAULT_SPEED);
        std::optional<double> escorting  = tte->estimateDuration(employeeLocation, appt.roomName, DEFAULT_SPEED);
        std::optional<double> travelBack = tte->estimateDuration(appt.roomName, startPos, DEFAULT_SPEED);
        double taskOverhead = 30.0 + 120.0; // scan + interaction + search 
        double buffer = 3 * 60.0; // 3 min safety buffer 
        
        assert(travelTo.has_value());
        assert(escorting.has_value());
        assert(travelBack.has_value());

        const double travelTime = travelTo.value() + escorting.value() + travelBack.value();
        const double startSeconds = appt.appointmentTime - (travelTime + taskOverhead + buffer);

        eventQueue.push(std::make_shared<MissionDispatchEvent>(static_cast<int>(startSeconds), appt));
    }

    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<IsSearching>("IsSearching");
    factory.registerNodeType<IsEscorting>("IsEscorting");
    factory.registerNodeType<HandleSearch>("HandleSearch");
    factory.registerNodeType<HandleEscort>("HandleEscort");
    factory.registerNodeType<HandleIdle>("HandleIdle");

    static const char* xml_text = R"(
     <root BTCPP_format="4">
         <BehaviorTree ID="MainTree">
            <Fallback>
                <Sequence>
                    <IsSearching/>
                    <HandleSearch/>
                </Sequence>
                <Sequence>
                    <IsEscorting/>
                    <HandleEscort/>
                </Sequence>
                <HandleIdle/>
            </Fallback>
         </BehaviorTree>
     </root>
    )";

    auto tree = std::make_shared<BT::Tree>(factory.createTreeFromText(xml_text));
    tree->rootBlackboard()->set("ctx", &ctx);
    ctx.behaviorTree = tree;

    while (!eventQueue.empty()) {
        auto e = eventQueue.top();
        eventQueue.pop();
        ctx.setTime(e->time);
        e->execute(ctx);
        std::cin.get();
    }

    // BT::StdCoutLogger logger(m_bt);
    // Timeline timelineView(model, maxSimTime);
    // timelineView.show();
    // BT::printTreeRecursively(tree.rootNode());
    // std::string xml_models = BT::writeTreeNodesModelXML(factory);
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
