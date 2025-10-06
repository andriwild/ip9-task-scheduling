#include <memory>
#include <queue>
#include <qt5/QtWidgets/QApplication>

#include "../include/MapView.h"
#include "../include/TimelineView.h"
#include "../include/Event.h"
#include "../include/Robot.h"
#include "../include/Dist.h"


int eventId = 1;
constexpr Point therapyLocation = {123,321};

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

double calculateDistance(const Point& p1, const Point& p2) {
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::unique_ptr<Event>> generateEscortEvents(const int startTime, const int endTime, const int mean) {
    int currentTime = startTime;
    std::vector<std::unique_ptr<Event>> events = {};
    do {
        const double randomTimeDelta = dist::expRand(mean);
        currentTime = currentTime + static_cast<int>(randomTimeDelta);
        if (currentTime < endTime) {
            EscortEvent escortEvent (eventId++, currentTime, generatePoint(), 0);
            events.push_back(std::make_unique<EscortEvent>(escortEvent));
        }
    } while (currentTime < endTime);
    return events;
}

std::vector<std::unique_ptr<Event>> generateSearchEvents(
    const std::vector<std::unique_ptr<Event>> &escortEvents,
    const double speed) {
    std::vector<std::unique_ptr<Event>> driveEvents = {};

    for(const auto &ev: escortEvents) {
        const Point searchPoint = generatePoint();
        const double distanceToTherapy = calculateDistance(searchPoint, ev->location);
        const int driveTimeToTherapy = static_cast<int>(distanceToTherapy / speed);

        SearchEvent se(eventId++,  ev->time - driveTimeToTherapy, searchPoint, 0);
        driveEvents.push_back(std::make_unique<SearchEvent>(se));
    }
    return driveEvents;
}

std::vector<std::unique_ptr<Event>> generateDriveEvents(
    const std::vector<std::unique_ptr<Event>> &escortEvents,
    const Point &initialPosition,
    const double speed,
    const Point dock) {
    Point lastPosition = initialPosition;
    std::vector<std::unique_ptr<Event>> driveEvents = {};

    for(const auto &escortEvent: escortEvents) {

        const Point searchPoint = generatePoint();
        const double distanceToTherapy = calculateDistance(searchPoint, escortEvent->location);
        const int driveTimeToTherapy = static_cast<int>(distanceToTherapy / speed);
        SearchEvent se(eventId++,  escortEvent->time - driveTimeToTherapy, searchPoint, 0);
        driveEvents.push_back(std::make_unique<SearchEvent>(se));

        // drive to search point
        const double distanceToSearchPoint = calculateDistance(lastPosition, searchPoint);
        const int driveTimeToSearchPoint = static_cast<int>(distanceToSearchPoint / speed);
        DriveStart ds { eventId++, escortEvent->time - (driveTimeToSearchPoint + driveTimeToTherapy), lastPosition };
        driveEvents.push_back(std::make_unique<DriveStart>(ds));
        DriveEnd de { eventId++, escortEvent->time - driveTimeToTherapy-1, searchPoint};
        driveEvents.push_back(std::make_unique<DriveEnd>(de));
        lastPosition = searchPoint;

        // bring person to therapy location
        DriveStart ds2 { eventId++, escortEvent->time - driveTimeToTherapy, lastPosition };
        driveEvents.push_back(std::make_unique<DriveStart>(ds2));
        DriveEnd de2 { eventId++, escortEvent->time -1, escortEvent->location};
        driveEvents.push_back(std::make_unique<DriveEnd>(de2));

        lastPosition = escortEvent->location;

        // go back to dock
        const double distanceToDock= calculateDistance(lastPosition, dock);
        const int driveTimeToDock= static_cast<int>(distanceToDock / speed);
        DriveStart ds3 { eventId++, escortEvent->time, lastPosition };
        driveEvents.push_back(std::make_unique<DriveStart>(ds3));
        DriveEnd de3 { eventId++, escortEvent->time + driveTimeToDock, dock};
        driveEvents.push_back(std::make_unique<DriveEnd>(de3));

        lastPosition = dock;
    }
    return driveEvents;
}

void sim() {
    constexpr int simTime = 3600;
    constexpr Point dock {0,0};
    Robot robot(dock);
    robot.setSpeed(3.0);

    auto escortEvents = generateEscortEvents(0, simTime, 2000);
    std::cout << escortEvents.size() << " EscortEvents created" << std::endl;

    auto driveEvents = generateDriveEvents(escortEvents, robot.getPosition(), robot.getSpeed(), dock);
    std::cout << driveEvents.size() << " DriveEvents created" << std::endl;

    for (auto &ev: escortEvents) {
        eventQueue.push(std::move(ev));
    }

    for (auto &ev: driveEvents) {
        eventQueue.push(std::move(ev));
    }

    Timeline timeline{simTime};
    timeline.draw();

    MapView mapView {};
    mapView.drawLocation(robot.getPosition(), "Init", Qt::blue);

    while(!eventQueue.empty()) {
        const auto &ev = eventQueue.top();

        if (dynamic_cast<DriveStart*>(ev.get())) {
            if (!robot.isDriving()) {
                robot.setDriving(ev->time);
            } else {
                std::cout << ev->eventId << " Robot already driving" << std::endl;
            }
        }
        else if (dynamic_cast<DriveEnd*>(ev.get())) {
            if (robot.isDriving()) {
                robot.stopDriving();
                mapView.driveFromTo(robot.getPosition(), ev->location);
                std::cout << robot.getPosition().x << ","<< robot.getPosition().y  << " - " << ev->location.x << "," << ev->location.y << std::endl;
                timeline.drawDrive(robot.getStartDrivingTime(), ev->time);
                robot.setPosition(ev->location);
            } else {
                std::cout << ev->eventId << " Robot is not driving - can't stop!"  << std::endl;
            }
        }
        else if (const auto escortEvent = dynamic_cast<EscortEvent*>(ev.get())) {
            timeline.drawEvent(*escortEvent);
            mapView.drawLocation(ev->location, std::to_string(ev->eventId));
        }
        else if (const auto searchEvent = dynamic_cast<SearchEvent*>(ev.get())) {
            timeline.drawEvent(*searchEvent, Qt::blue);
            mapView.drawLocation(ev->location, std::to_string(ev->eventId), Qt::blue);
        }

        eventQueue.pop();
    }

    timeline.show();
    mapView.show();
    QApplication::exec();
}


int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    sim();
    //dist::show();
    return 0;
}
