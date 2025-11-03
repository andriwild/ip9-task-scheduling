#include "model/event.h"
#include "datastructure/graph.h"
#include "view/map_view.h"
#include "model/robot.h"
#include "algo/rnd.h"
#include "datastructure/tree.h"
#include "model/planner.h"
#include "model/simulation.h"
#include "util/tree_composer.h"
#include "view/timeline_view.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include <QtConcurrent/QtConcurrent>
#include <behaviortree_cpp/blackboard.h>

#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviour/nodes/event_handler.h"


constexpr int maxSimTime = 3600;


int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    Graph graph;
    std::vector nodes = {
        Node(359.183, 202.229),
        Node(311.896, 227.382),
        Node(362.202, 236.437),
        Node(406.471, 257.566),
        Node(461.807, 280.706),
        Node(549.339, 330.006),
        Node(504.064, 394.398),
        Node(545.315, 395.404),
        Node(610.713, 354.153),
        Node(719.373, 419.55),
        Node(824.009, 500.04),
        Node(367.232, 170.034),
        Node(317.933, 118.722),
        Node(371.257, 71.4343),
        Node(424.581, 105.642),
        Node(256.56, 260.584),
        Node(206.254, 384.336),
        Node(130.795, 365.22),
        Node(230.401, 291.774),
        Node(394.398, 206.254)
    };
    for (auto n: nodes) {
        graph.addNode(n);
    }
    graph.addEdge(0, 1);
    graph.addEdge(0, 2);
    graph.addEdge(1, 2);
    graph.addEdge(0, 11);
    graph.addEdge(11, 12);
    graph.addEdge(11, 14);
    graph.addEdge(13, 14);
    graph.addEdge(13, 12);
    graph.addEdge(1, 15);
    graph.addEdge(18, 15);
    graph.addEdge(18, 17);
    graph.addEdge(18, 16);
    graph.addEdge(3, 2);
    graph.addEdge(3, 4);
    graph.addEdge(4, 5);
    graph.addEdge(5, 8);
    graph.addEdge(9, 8);
    graph.addEdge(9, 10);
    graph.addEdge(5, 6);
    graph.addEdge(5, 7);
    graph.addEdge(6, 7);
    graph.addEdge(19, 2);
    graph.addEdge(19, 0);

    // TODO: expectation that fist search location has biggest change to find the person
    util::PersonData personData;
    personData[0].push_back(6);
    personData[0].push_back(10);
    personData[0].push_back(12);
    personData[1].push_back(13);
    personData[1].push_back(12);
    personData[1].push_back(14);
    personData[2].push_back(17);
    personData[2].push_back(18);
    personData[3].push_back(4);

    constexpr int robotPosition = 0;
    constexpr int dock = 11;

    Robot robot(robotPosition, dock);
    EV::Tree<SimulationEvent> eventTree;

    Planner planner(graph, robot.getSpeed());

    auto root = eventTree.createRoot(std::make_unique<SimulationRoot>(0));
    root->addSubtree(planner.escortSequence(700, personData,0, 16, dock, dock).releaseRoot());
    // root->addSubtree(planner.escortSequence(1500, personData,1,  8, dock, dock).releaseRoot());
    // root->addSubtree(planner.escortSequence(2000, personData,2,  13, dock, dock).releaseRoot());
    // root->addSubtree(planner.escortSequence( 400, personData,2,  7, dock, dock).releaseRoot());

    // EV::Tree<SimulationEvent> dockTree;
    // auto dockTreeRoot = dockTree.createRoot(std::make_unique<Tour>(100, "Tour"));
    // planner.tour(dockTreeRoot, 0, 10);
    // root->addSubtree(dockTree.releaseRoot());

    std::cout << eventTree << std::endl;

    BT::Tree m_bt;
    SimTime m_simTime;
    ReadOnlyClock roClock(&m_simTime);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<DriveStart>("DriveStart");
    factory.registerNodeType<NavigateToPoint>("NavigateToPoint", &robot, &graph, &roClock);
    factory.registerNodeType<EventHandler>("EventHandler", &eventTree, &roClock);
    factory.registerNodeType<SayHello>("SayHello");
    factory.registerNodeType<LogNode>("LoggerNode");

    m_bt = factory.createTreeFromFile("../tree.xml");
    // m_bt.rootBlackboard().get()->debugMessage();
    m_bt.rootBlackboard().get()->set("queue", std::vector {1,2});
    m_bt.rootBlackboard().get()->set("task", "drive");
    m_bt.rootBlackboard().get()->set("msg", "default string");
    m_bt.rootBlackboard().get()->set("path", std::vector<int>{1,2,3});

    // BT::StdCoutLogger logger(m_bt);

    BT::Groot2Publisher publisher(m_bt);
    Simulation model(graph, &robot, eventTree, personData, m_bt, &m_simTime);
    MapView mapView(model);

    Timeline timelineView(model, maxSimTime);
    timelineView.show();
    //BT::printTreeRecursively(tree.rootNode());
    //
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::cout << xml_models << std::endl;

    mapView.show();

    QtConcurrent::run([&] {
        for (int i = 0; i < maxSimTime; ++i) {
            model.simStep();

            //auto msg = m_bt.rootBlackboard().get()->getEntry("msg");
            m_bt.sleep(std::chrono::milliseconds(100));
            // std::cout << "simStep: " << m_simTime.getTime() << std::endl;
            //
            // m_bt.rootBlackboard().get()->set("goal", 16);
            // auto keys = m_bt.rootBlackboard().get()->getKeys();
            // auto goal = m_bt.rootBlackboard().get()->getEntry("goal");
            // auto p = goal.get()->value;
            // std::cout << p.cast<std::string>() << std::endl;
            // for (const auto k: keys) {
            //     std::cout << k << std::endl;
            // }
        }
    });
    QApplication::exec();
    return 0;
}