#include <memory>
#include <queue>
#include <qt5/QtWidgets/QApplication>

#include "../include/map_view.h"
#include "../include/timeline_view.h"
#include "../include/event.h"
#include "../include/robot.h"
#include "../include/rnd.h"
#include "../include/util.h"

int eventId = 1;
int personId = 1;

constexpr Point therapyLocation = {50,50};

constexpr int maxSimTime = 3600;

struct CompareEventTime {
    bool operator()(const std::unique_ptr<event> &a, const std::unique_ptr<event> &b) const {
        return a->time > b->time;
    }
};

std::priority_queue<
    std::unique_ptr<event>,
    std::vector<std::unique_ptr<event>>,
    CompareEventTime
> eventQueue {};



std::vector<Point> searchForPerson(const int pId, const Point escortLocation, const double radius = 100) {
    std::vector<Point> points(3);
    std::ranges::generate(
        points,
        [escortLocation, radius] {
            return rnd::generatePointAround(escortLocation, radius);
        });
    return points;
}

template<typename Generator>
std::vector<std::unique_ptr<event>> generateEscortEvents(
    const int startTime,
    const int endTime,
    const Point &center,
    Generator rndGen)
{
    std::vector<std::unique_ptr<event>> events = {};
    int currentTime = startTime + rndGen();
    while (currentTime < endTime){
        auto escortEvent = std::make_unique<event>(
            eventId++, currentTime, rnd::generatePointAround(center, 200.0), personId++
            );
        events.push_back(std::move(escortEvent));
        currentTime = currentTime + static_cast<int>(rndGen());
    }
    return events;
}

void planSearchPersonEvents(const std::vector<std::unique_ptr<event>> &escortEvents, const double searchRadius) {
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
        totalDistance += util::calculateDistance(currentPos, nextPos);
        currentPos = nextPos;
        pathCopy.pop();
    }
    totalDistance += util::calculateDistance(currentPos, finalDestination);
    return static_cast<int>(totalDistance / speed);
}

std::queue<std::shared_ptr<SearchEvent>> shortestPath(
    const Point &start,
    const std::vector<std::shared_ptr<SearchEvent>> & points
    ) {
    // TODO: implement shortest path
    std::queue<std::shared_ptr<SearchEvent>> events = {};
    for (auto p: points) {
        events.push(p);
    }
    return events;
}

void runSimulation(const Timeline &timeline, const map_view &mapView, robot &robot) {
    // robot simulation
    constexpr double personFoundProbability = 0.8;
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
                const auto searchEvent = path.front();
                const int driveTime = util::calcDriveTime(robot.getPosition(), searchEvent->location, robot.getSpeed());
                mapView.drawPath(robot.getPosition(), searchEvent->location);
                std::string label = std::to_string(event->eventId) + "." + std::to_string(searchPointCounter++);
                timeline.drawEvent(startDriveTime + driveTime, label, Qt::darkRed, false);
                timeline.drawDrive(startDriveTime, driveTime);
                robot.setPosition(searchEvent->location);
                startDriveTime += driveTime;
                path.pop();

                // found person?
                if (rnd::uniRand() > personFoundProbability) {
                    personFound = true;
                }
            }
            // escort person to therapy
            const int driveTime = util::calcDriveTime(robot.getPosition(), event->location, robot.getSpeed());
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
}

void drawEvents(const Timeline &timeline, const map_view &mapView, std::vector<std::unique_ptr<event>> &escortEvents) {
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
}

void sim(const double searchLocationRadius, const double lambda) {
    constexpr Point dock { 0,0 };
    constexpr double speed = 3.0;
    robot robot(dock, speed);

    Timeline timeline{ maxSimTime };
    timeline.draw();

    map_view mapView {};
    mapView.drawLocation(robot.getPosition(), "Init", Qt::blue);

    auto genRnd = [lambda] { return rnd::rnd(std::poisson_distribution<>(lambda)); };
    auto escortEvents = generateEscortEvents(0, maxSimTime, dock, genRnd);
    planSearchPersonEvents(escortEvents, searchLocationRadius);

    drawEvents(timeline, mapView, escortEvents);
    runSimulation(timeline, mapView, robot);

    timeline.show();
    mapView.show();
    QApplication::exec();
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    sim(100, 200);
    //dist::show();
    return 0;
}