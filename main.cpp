#include "model/event.h"
#include "datastructure/graph.h"
#include "view/map_view.h"
#include "model/robot.h"
#include "algo/rnd.h"
#include "datastructure/tree.h"
#include "model/planner.h"
#include "model/simulation.h"
#include "view/timeline_view.h"

//#include "behaviortree_cpp/bt_factory.h"

int eventId = 1;
int personId = 1;

constexpr int maxSimTime = 3600;

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    // DBClient db {"",""};
    //
    // if (argc == 3) {
    //     const std::string dbUser = argv[1];
    //     const std::string dbPassword= argv[2];
    //     db.setCredentials(dbUser, dbPassword) ;
    //
    //     if (db.init()) {
    //         Log::i("DB connected");
    //     }
    // }
    // BT::BehaviorTreeFactory factory;
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
    personData[0].push_back(4);
    personData[0].push_back(10);
    personData[0].push_back(12);
    personData[1].push_back(14);
    personData[1].push_back(13);

    const int robotPosition = 0;
    const int dock = 11;
    const int coffeMachineId = 16;

    Robot robot(robotPosition, dock);

    Tree<SimulationEvent> eventTree;
    EventQueue events;

    auto root = eventTree.createRoot(std::make_unique<Tour>());
    // auto n1 = root->addChild(std::make_unique<MeetingEvent>(1000, coffeMachineId, 0));
    // auto n2 = root->addChild(std::make_unique<MeetingEvent>(1000, coffeMachineId, 0));
    // n1->addChild(std::make_unique<RobotDriveStartEvent>(DRIVE, 1800, 100));
    // n1->addChild(std::make_unique<RobotDriveEndEvent>(1900, 1));


    //events.addEvent<PersonEscortEvent>(100, coffeMachineId, 0);
    // events.addEvent<MeetingEvent>(1000, coffeMachineId, 0);
    // events.addEvent<MeetingEvent>(2000, 7, 1);

    Planner planner(graph, events);
    //planner.process(robot.getPosition(), personData, robot.getSpeed());

    auto tourEnd = root->getRightMostLeaf()->getValue()->getTime();
    auto tree = planner.tour(tourEnd, robot.getPosition(), 10, robot.getSpeed());
    root->addSubtree(tree.getRoot());

    auto tourEnd2 = root->getRightMostLeaf()->getValue()->getTime();
    auto tree2 = planner.tour(tourEnd2, 10, 11, robot.getSpeed());
    root->addSubtree(tree2.getRoot());


    Simulation model(graph, robot, eventTree, personData);
    MapView mapView(model);
    Timeline timelineView(model, maxSimTime);

    timelineView.show();
    mapView.show();
    QApplication::exec();
    return 0;
}
