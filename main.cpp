#include "model/event.h"
#include "datastructure/graph.h"
#include "view/map_view.h"
#include "model/robot.h"
#include "algo/rnd.h"
#include "model/planner.h"
#include "model/simulation.h"
#include "view/timeline_view.h"

//#include "behaviortree_cpp/bt_factory.h"

int eventId = 1;
int personId = 1;

constexpr int maxSimTime = 3600;

// struct CompareEventTime {
//     bool operator()(const std::unique_ptr<EscortEvent> &a, const std::unique_ptr<EscortEvent> &b) const {
//         return a->time > b->time;
//     }
// };
//
// std::priority_queue<
//     std::unique_ptr<EscortEvent>,
//     std::vector<std::unique_ptr<EscortEvent>>,
//     CompareEventTime
// > eventQueue {};
//
// std::vector<Node> searchForPerson(const int pId, const Node escortLocation, const double radius = 100) {
//     std::cout << "[searchForPerson] generate search points for person " << pId << std::endl;
//     std::vector<Node> nodes;
//     std::ranges::generate(
//         nodes,
//         [escortLocation, radius] {
//             return rnd::generateNodeAround(escortLocation, radius);
//         });
//     return nodes;
// }
//
// template<typename Generator>
// std::vector<std::unique_ptr<EscortEvent>> generateEscortEvents(
//     const int startTime,
//     const int endTime,
//     const Node &center,
//     Generator rndGen
//     ) {
//     std::cout << "[generateEscortEvents] generate escort locations" << std::endl;
//     std::vector<std::unique_ptr<EscortEvent>> events = {};
//     int currentTime = startTime + rndGen();
//     while (currentTime < endTime){
//         EscortEvent event(eventId++, currentTime, rnd::generateNodeAround(center, 200.0), personId++);
//         auto escortEvent = std::make_unique<EscortEvent>(event);
//         events.push_back(std::move(escortEvent));
//         currentTime = currentTime + static_cast<int>(rndGen());
//     }
//     return events;
// }
//
// void planSearchPersonEvents(const std::vector<std::unique_ptr<EscortEvent>> &escortEvents, const double searchRadius) {
//     std::cout << "[planSearchPersonEvents] add search locations to escort event " << std::endl;
//     for(const auto &escortEvent: escortEvents) {
//         auto searchLocations = searchForPerson(escortEvent->personId, escortEvent->location, searchRadius);
//         for (Node sl: searchLocations) {
//             auto event = std::make_unique<SearchEvent>(eventId++, sl);
//             escortEvent->children.push_back(std::move(event));
//         }
//     }
// }
//
// int calcMaxTourTime(
//     const Node &robotPos,
//     const std::queue<std::shared_ptr<SearchEvent>> &path,
//     const Node& finalDestination,
//     const double speed
//     ) {
//     assert(!path.empty());
//
//     double totalDistance = 0;
//     auto pathCopy = path;
//     Node currentPos = robotPos;
//
//     while (!pathCopy.empty()) {
//         Node nextPos = pathCopy.front()->location;
//         totalDistance += util::calculateDistance(currentPos, nextPos);
//         currentPos = nextPos;
//         pathCopy.pop();
//     }
//     totalDistance += util::calculateDistance(currentPos, finalDestination);
//     return static_cast<int>(totalDistance / speed);
// }
//
// std::queue<std::shared_ptr<SearchEvent>> shortestPath(
//     const Node &start,
//     const std::vector<std::shared_ptr<SearchEvent>> & points
//     ) {
//     // TODO: implement shortest path
//     std::queue<std::shared_ptr<SearchEvent>> events = {};
//     for (auto const& p: points) {
//         events.push(p);
//     }
//     return events;
// }
//
// void runSimulation(
//     const Timeline &timeline,
//     const MapView &mapView,
//     Robot &robot,
//     const bool useDock,
//     const double personFoundProbability
//     ) {
//     // robot simulation
//     int simTime = 0;
//     while (!eventQueue.empty()) {
//         const auto &event = eventQueue.top();
//         auto path = shortestPath(robot.getPosition(), event->children);
//         const int tourTime = calcMaxTourTime(robot.getPosition(), path, event->location, robot.getSpeed());
//         int startDriveTime = event->time - tourTime;
//
//         if (startDriveTime > simTime) {
//             // search for person
//             bool personFound = false;
//             int searchNodeCounter = 1;
//             while (!personFound && !path.empty()) {
//                 const auto searchEvent = path.front();
//                 const int driveTime = util::calcDriveTime(robot.getPosition(), searchEvent->location, robot.getSpeed());
//                 mapView.drawPath(robot.getPosition(), searchEvent->location);
//                 std::string label = std::to_string(event->eventId) + "." + std::to_string(searchNodeCounter++);
//                 timeline.drawEvent(startDriveTime + driveTime, label, Qt::darkRed, false);
//                 timeline.drawDrive(startDriveTime, driveTime);
//                 robot.setPosition(searchEvent->location);
//                 startDriveTime += driveTime;
//                 path.pop();
//
//                 // found person?
//                 if (rnd::uniRand() < personFoundProbability) {
//                     personFound = true;
//                 }
//             }
//             // escort person to therapy
//             const int driveTime = util::calcDriveTime(robot.getPosition(), event->location, robot.getSpeed());
//             startDriveTime = event->time - driveTime;
//             mapView.drawPath(robot.getPosition(), event->location, Qt::darkGreen);
//             timeline.drawDrive(startDriveTime, driveTime, Qt::darkGreen);
//             robot.setPosition(event->location);
//
//             if (useDock) {
//                 const int driveToDockTime = util::calcDriveTime(robot.getPosition(), robot.getDock(), robot.getSpeed());
//                 mapView.drawPath(robot.getPosition(), robot.getDock(), Qt::blue);
//                 timeline.drawDrive(event->time, driveToDockTime, Qt::blue);
//                 timeline.drawEvent(event->time + driveToDockTime, "dock", Qt::blue);
//                 robot.setPosition(robot.getDock());
//                 simTime = event->time + driveToDockTime;
//             } else {
//                 simTime = event->time;
//             }
//
//         } else {
//             std::cout << "[runSimulation] Skip event "<< event->eventId << std::endl;
//         }
//         eventQueue.pop();
//     }
// }
//
// void drawEvents(const Timeline &timeline, const MapView &mapView, std::vector<std::unique_ptr<EscortEvent>> &escortEvents) {
//     for (auto &e: escortEvents) {
//         std::string label = std::to_string(e->eventId);
//         timeline.drawEvent(e->time, label, Qt::red, true);
//         mapView.drawLocation(e->location, label);
//         int searchNodeId = 1;
//
//         for (const auto &se: e->children) {
//             std::string searchLabel = label + "." + std::to_string(searchNodeId++);
//             mapView.drawLocation(se->location, searchLabel);
//         }
//         eventQueue.push(std::move(e));
//     }
// }
//
//
// Node transform(const Node&p, const double mapHeight) {
//     double resolution  = 20;
//     double originX = -7.36;
//     double originY = -17.1;
//     double newOriginX = -(originX * resolution);
//     double newOriginY = -(originY * resolution);
//     return {p.x * resolution + newOriginX, mapHeight - (p.y * resolution + newOriginY)};
// }
//
// void sim(Graph &graph, const double searchLocationRadius, const double lambda, bool useDock = false, const double personFindProb = 0.8) {
//     constexpr double speed = 3.0;
//     const auto dock = graph.getNode(0);
//     const auto startPos = graph.getNode(1);
//     assert(dock.has_value());
//     assert(startPos.has_value());
//     Robot robot(startPos.value(), dock.value(), speed);
//
//     Timeline timeline{ maxSimTime };
//     timeline.draw();
//
//     MapView mapView {};
//     mapView.drawGraph(graph);
//
//     int startNode = 0;
//     int targetNode = 10;
//     auto r = graph.dijkstra(startNode);
//     auto r2 = r.shortestPath(targetNode);
//     Node lastNode = graph.getNode(startNode).value();
//     std::cout << "shortest path:" << std::endl;
//     for (auto i: r2) {
//         std::cout << i << " -> ";
//         auto n = graph.getNode(i).value();
//         mapView.drawPath(lastNode, n, Qt::blue);
//         lastNode = n;
//     }
//     std::cout << std::endl;
//
//
//    // auto entrances = db.entrances();
//     // if (entrances.has_value()) {
//     //     for (Node p: entrances.value()) {
//     //         p = transform(p, mapView.height());
//     //         mapView.drawLocation(p, p.to_string());
//     //     }
//     // }
//     // mapView.drawLocation(robot.getPosition(), "Init", Qt::blue);
//
//     // auto genRnd = [lambda] { return rnd::rnd(std::poisson_distribution<>(lambda)); };
//     // auto escortEvents = generateEscortEvents(0, maxSimTime, dock, genRnd);
//     // planSearchPersonEvents(escortEvents, searchLocationRadius);
//     //
//     // drawEvents(timeline, mapView, escortEvents);
//     // runSimulation(timeline, mapView, robot, useDock, personFindProb);
//
//     //timeline.show();
//     mapView.show();
//     QApplication::exec();
// }

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

    EventQueue events;
    //events.addEvent<PersonEscortEvent>(100, coffeMachineId, 0);
    events.addEvent<MeetingEvent>(1000, coffeMachineId, 0);
    events.addEvent<MeetingEvent>(2000, 7, 1);

    Planner planner(graph, events);
    planner.process(robot, personData);

    for (auto ev: events.getAllEvents()) {
            if (auto* event = dynamic_cast<MeetingEvent*>(ev)) {
                std::cout << event->getTime() << " MeetingEvent with Person: " << event->getPersonId() << std::endl;

            } else if (auto* startEvent = dynamic_cast<RobotDriveStartEvent*>(ev)) {
                std::cout << startEvent->getTime() << " RobotDriveStartEvent  to " << startEvent->m_targetId << " arrivalTime: "<< startEvent->m_expectedArrival <<" [" << startEvent->m_task << "]" << std::endl;

            } else if (auto* endEvent = dynamic_cast<RobotDriveEndEvent*>(ev)) {
                std::cout << endEvent->getTime() << " RobotDriveEndEvent" << std::endl;

            }
    }



    Simulation model(graph, robot, events, personData);
    MapView mapView(model);
    Timeline timelineView(model, maxSimTime);

    timelineView.show();
    mapView.show();
    QApplication::exec();
    return 0;
}
