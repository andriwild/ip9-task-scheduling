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
    Tree<SimulationEvent> eventTree;

    Planner planner(graph, robot.getSpeed());
    auto meetingTree1 = planner.escortSequence(1200, personData,0, 16, dock, dock);
    auto meetingTree2 = planner.escortSequence(1500, personData,1,  8, dock, dock);
    auto meetingTree3 = planner.escortSequence(2000, personData,2,  13, dock, dock);
    auto meetingTree4 = planner.escortSequence( 400, personData,2,  7, dock, dock);

    auto root = eventTree.createRoot(std::make_unique<SimulationRoot>(0));
    root->addSubtree(meetingTree1.releaseRoot());
    root->addSubtree(meetingTree2.releaseRoot());
    root->addSubtree(meetingTree3.releaseRoot());
    root->addSubtree(meetingTree4.releaseRoot());

    std::cout << eventTree << std::endl;

    Simulation model(graph, robot, eventTree, personData);
    MapView mapView(model);
    Timeline timelineView(model, maxSimTime);

    timelineView.show();
    //mapView.show();
    QApplication::exec();
    return 0;
}