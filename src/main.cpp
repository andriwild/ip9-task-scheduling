#include <memory>
#include <queue>
#include <qt5/QtWidgets/QApplication>

#include "../include/MapView.h"
#include "../include/TimelineView.h"
#include "../include/Event.h"
#include "../include/Robot.h"
#include "../include/Dist.h"

int eventId = 1;
int personId = 1;

constexpr Point therapyLocation = {123,321};

constexpr int maxSimTime = 3600;
constexpr Point dock {0,0};
Robot robot(dock, 100.0);

struct CompareEventTime {
    bool operator()(const std::unique_ptr<Event> &a, const std::unique_ptr<Event> &b) const {
        return a->time > b->time;
    }
};

std::priority_queue<
    std::unique_ptr<Event>,
    std::vector<std::unique_ptr<Event>>,
    CompareEventTime
> eventQueue {};

Point generatePoint() {
    const auto values = dist::dist(std::normal_distribution<>(200, 100), 2);
    return Point{values[0], values[1]};
}

Point generatePointAround(const Point center, const double radius) {
    const auto offsets = dist::dist(std::normal_distribution<>(0, radius), 2);
    return Point{center.x + offsets[0], center.y + offsets[1]};
}

std::vector<Point> searchForPerson(const int pId, const Point escortLocation, const double radius = 100) {
    std::vector<Point> points(3);
    std::ranges::generate(
        points,
        [escortLocation, radius] {
            return generatePointAround(escortLocation, radius);
        });
    return points;
}

template<typename Generator>
std::vector<std::unique_ptr<Event>> generateEscortEvents(const int startTime, const int endTime, Generator rndGen) {
    std::vector<std::unique_ptr<Event>> events = {};
    int currentTime = startTime + rndGen();
    while (currentTime < endTime){
        auto escortEvent = std::make_unique<Event>(eventId++, currentTime, generatePointAround(dock, 200.0), personId++);
        events.push_back(std::move(escortEvent));
        currentTime = currentTime + static_cast<int>(rndGen());
    }
    return events;
}

void planSearchPersonEvents(const std::vector<std::unique_ptr<Event>> &escortEvents, const double searchRadius) {
    for(const auto &escortEvent: escortEvents) {
        auto searchLocations = searchForPerson(escortEvent->personId, escortEvent->location, searchRadius);
        for (Point sl: searchLocations) {
            auto event = std::make_unique<SearchEvent>(eventId++, sl);
            escortEvent->children.push_back(std::move(event));
        }
    }
}

int calcMaxTourTime(
    const Point &robotPos,
    const std::queue<std::shared_ptr<SearchEvent>> &path,
    const Point& finalDestination,
    const double speed
    ) {
    if (path.empty()) {
        return 0;
    }
    double totalDistance = 0;
    auto pathCopy = path;
    Point currentPos = robotPos;

    while (!pathCopy.empty()) {
        Point nextPos = pathCopy.front()->location;
        totalDistance += Robot::calculateDistance(currentPos, nextPos);
        currentPos = nextPos;
        pathCopy.pop();
    }
    totalDistance += Robot::calculateDistance(currentPos, finalDestination);
    return static_cast<int>(totalDistance / speed);
}

std::queue<std::shared_ptr<SearchEvent>> shortestPath(
    const Point &start,
    const std::vector<std::shared_ptr<SearchEvent>> & points
    ) {
    std::queue<std::shared_ptr<SearchEvent>> events = {};
    for (auto p: points) {
        events.push(p);
    }
    return events;
}

void sim(const double searchLocationRadius, const double lambda) {
    Timeline timeline{maxSimTime};
    timeline.draw();

    MapView mapView {};
    mapView.drawLocation(robot.getPosition(), "Init", Qt::blue);


    auto genRnd = [lambda] { return dist::rnd(std::poisson_distribution<>(lambda)); };
    auto escortEvents = generateEscortEvents(0, maxSimTime, genRnd);
    planSearchPersonEvents(escortEvents, searchLocationRadius);

    // draw events
    for (auto &e: escortEvents) {
        std::string label = std::to_string(e->eventId);
        timeline.drawEvent(e->time, label, Qt::red, true);
        mapView.drawLocation(e->location, label);
        int searchPointId = 1;

        for (const auto &se: e->children) {
            std::string searchLabel = label + "." + std::to_string(searchPointId++);
            mapView.drawLocation(se->location, searchLabel);
        }
        eventQueue.push(std::move(e));
    }

    // robot simulation
    int simTime = 0;
    while (!eventQueue.empty()) {
        const auto &event = eventQueue.top();
        auto path = shortestPath(robot.getPosition(), event->children);
        const int tourTime = calcMaxTourTime(robot.getPosition(), path, event->location, robot.getSpeed());
        int startDriveTime = event->time - tourTime;

        if (startDriveTime > simTime) {
            // search for person
            bool personFound = false;
            int searchPointCounter = 1;
            while (!personFound && !path.empty()) {
                const auto se = path.front();
                const int driveTime = robot.calcDriveTime(se->location);
                mapView.drawPath(robot.getPosition(), se->location);
                std::string label = std::to_string(event->eventId) + "." + std::to_string(searchPointCounter++);
                timeline.drawEvent(startDriveTime + driveTime, label, Qt::darkRed, false);
                timeline.drawDrive(startDriveTime, driveTime);
                robot.setPosition(se->location);
                startDriveTime += driveTime;
                path.pop();

                // found person?
                if (dist::uniRand() > 0.8) {
                    personFound = true;
                }
            }
            // escort person to therapy
            const int driveTime = robot.calcDriveTime(event->location);
            startDriveTime = event->time - driveTime;
            mapView.drawPath(robot.getPosition(), event->location, Qt::darkGreen);
            timeline.drawDrive(startDriveTime, driveTime, Qt::darkGreen);
            robot.setPosition(event->location);

            simTime = event->time;
        } else {

            std::cout << "Can't reach goal!" << std::endl;
        }
        eventQueue.pop();
    }

    timeline.show();
    mapView.show();
    QApplication::exec();
}


int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    sim(500, 200);
    //dist::show();
    return 0;
}