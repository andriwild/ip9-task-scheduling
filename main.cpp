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
#include "behaviour/dummy.h"
#include "behaviour/nodes/event_handler.h"
#include "behaviour/nodes/robot_state.h"
#include "util/grid.h"
#include "util/map_loader.h"
#include "sdf/sdf_parser.h"

#include <gz/transport.hh>
#include <gz/msgs.hh>

constexpr int maxSimTime = 3600;
constexpr double GRID_SIZE = 50;
constexpr double SEGMENT_SCALE = 100.0;
constexpr bool SHOW_VISITED = false;

const std::string RES_PATH = "/home/andri/repos/ip9-task-scheduling/resources/";
const std::string MAP_FILENAME = "IMVS_data.bin";

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);


    std::vector<VisLib::Segment> imvsSegments = {};
    SdfParser sdfParser;
    //sdfParser.loadSdf(RES_PATH + "imvs.sdf", imvsSegments);
    std::vector<VisLib::Segment> segments;
    std::vector<VisLib::Point> points;
    sdfParser.extractWallSegments(RES_PATH + "imvs_l.sdf", segments, points);
    for (auto s: segments) {
        std::cout << s << "\n";
    }



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

    EV::Tree<SimulationEvent> eventTree;

    //VisLib::readFile(RES_PATH + MAP_FILENAME, segments, points);

    for (auto& s: segments) {
        s.m_points[0].m_x *= SEGMENT_SCALE;
        s.m_points[0].m_y *= SEGMENT_SCALE;
        s.m_points[1].m_x *= SEGMENT_SCALE;
        s.m_points[1].m_y *= SEGMENT_SCALE;
    }

    BT::Tree m_bt;
    SimTime m_simTime;
    ReadOnlyClock roClock(&m_simTime);

    Graph graph;

    auto map = new Map(segments, points);

    QRectF bounds = map->boundingRect();
    const int h = static_cast<int>(bounds.height());
    const int w = static_cast<int>(bounds.width());
    double offsetX = bounds.x();
    double offsetY = bounds.y();

    std::vector<std::vector<Node>> girdLines;
    // createGridGraph(GRID_SIZE, offsetX, offsetY, h, w, graph, girdLines);
    // connectAllGridNodes(graph, girdLines);
    // auto visitedNodes = removeIntersectingEdges(graph, girdLines, segments, offsetX, offsetY, GRID_SIZE);

    int robotPosition = getNearestNode(graph, -930, -980);
    int dock = getNearestNode(graph, -826, -922);

    Robot robot(robotPosition, dock);
    Simulation model(graph, &robot, eventTree, personData, m_bt, &m_simTime);
    MapView mapView(model, map);
    // if (SHOW_VISITED) {
    //     for (auto n: visitedNodes) {
    //         mapView.drawLocation(n, "", Qt::green, N, 20, -1);
    //     }
    // }
    mapView.show();

    Planner planner(graph, robot.getSpeed());

    auto root = eventTree.createRoot(std::make_unique<SimulationRoot>(0));
    //root->addSubtree(planner.escortSequence(1000, personData,0, 16, robotPosition, dock).releaseRoot());
    // root->addSubtree(planner.escortSequence(1500, personData,1,  8, dock, dock).releaseRoot());
    // root->addSubtree(planner.escortSequence(2000, personData,2,  13, dock, dock).releaseRoot());
    // root->addSubtree(planner.escortSequence( 400, personData,2,  7, dock, dock).releaseRoot());

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
    QApplication::exec();
    return 0;
}