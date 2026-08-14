/*
 * An employee in the building: static profile plus the per-day
 * schedule (arrival, lunch, departure) and a room transition matrix.
 * Owns an id-keyed RNG stream so schedules stay reproducible
 * independently of the robot.
 * Copy is not allowed because of the rng stream per person.
 *
 */
#pragma once

#include <algorithm>
#include <iomanip>
#include <random>
#include <string>
#include <vector>

#include "../util/rnd.h"
#include "room.h"
#include "../util/constants.h"

namespace des {

// TODO: add to sim config file
struct StayDurationConfig {
    double officeMin = 60 * 10;
    double officeMax = ONE_HOUR * 2;
    double classroomMin = 60 * 45;
    double classroomMax = 60 * 90;
    double meetingMin = 60 * 30;
    double meetingMax = 60 * 90;
    double kitchenMin = 30;
    double kitchenMax = ONE_HOUR / 2.0;
    double toiletMu = 4.8;
    double toiletSigma = 0.7;
    double spaceMin = ONE_HOUR / 2.0;
    double spaceMax = ONE_HOUR * 4;
    double miscMin = 60;
    double miscMax = 60 * 5;
    double accessMin = 60;
    double accessMax = 120;
};

class Person {
public:
    Person() = default;
    Person(const Person&) = delete;
    Person& operator=(const Person&) = delete;
    Person(Person&&) = default;
    Person& operator=(Person&&) = default;

    int id{};
    std::string firstName;
    std::string lastName;
    std::string sex;
    std::string workplace;
    std::string color;
    bool busy = false;
    int arrivalTime{};
    int departureTime{};
    int lunchTime{};
    double lunchDuration{};
    bool lunchPending = false;
    std::vector<std::string> roomLabels;
    std::vector<std::vector<double>> transitionMatrix;
    // TODO: static function instead of a member
    StayDurationConfig stayDuration;
    std::mt19937 rng{};

    void reseed(const unsigned int seed) {
        rng.seed(seed + static_cast<unsigned int>(id));
    }

    double getStayDuration(const RoomType roomType, std::mt19937& rng) const {
        switch (roomType) {
            case RoomType::OFFICE:
                return rnd::uni(rng, stayDuration.officeMin, stayDuration.officeMax);
            case RoomType::CLASSROOM:
                return rnd::uni(rng, stayDuration.classroomMin, stayDuration.classroomMax);
            case RoomType::MEETING:
                return rnd::uni(rng, stayDuration.meetingMin, stayDuration.meetingMax);
            case RoomType::TOILET:
                return rnd::logNormal(rng, stayDuration.toiletMu, stayDuration.toiletSigma);
            case RoomType::KITCHEN:
                return rnd::uni(rng, stayDuration.kitchenMin, stayDuration.kitchenMax);
            case RoomType::ACCESS:
                return rnd::uni(rng, stayDuration.accessMin, stayDuration.accessMax);
            case RoomType::SPACE:
                return rnd::uni(rng, stayDuration.spaceMin, stayDuration.spaceMax);
            case RoomType::MISC:
                return rnd::uni(rng, stayDuration.miscMin, stayDuration.miscMax);
        }
        return rnd::uni(rng, stayDuration.miscMin, stayDuration.miscMax);
    }

    friend std::ostream& operator<<(std::ostream& os, const Person& p) {
        os << "-------------------------------------------\n"
            << "ID: " << p.id << " | Name: " << p.firstName << " " << p.lastName << "\n"
            << "sex: " << p.sex << "\n"
            << "workplace: " << p.workplace << "\n";
        return os;
    }
};

}  // namespace des
