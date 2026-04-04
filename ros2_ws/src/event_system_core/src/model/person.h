#pragma once

#include <iomanip>
#include <random>
#include <regex>
#include <string>
#include <vector>

#include "../util/rnd.h"

namespace des {

enum class RoomType {
    WORKPLACE,
    TOILET,
    KITCHEN,
    OTHER
};

inline RoomType parseRoomName(const std::string& roomName) {
    if (roomName.find("Toilet") != std::string::npos) return RoomType::TOILET;
    if (roomName.find("Kitchen") != std::string::npos) return RoomType::KITCHEN;
    static const std::regex workplacePattern(R"(5\.2[A-Z]\d+)");
    if (std::regex_match(roomName, workplacePattern)) return RoomType::WORKPLACE;
    return RoomType::OTHER;
}

struct StayDurationConfig {
    double workplaceMin = 60 * 10;
    double workplaceMax = 3600 * 2;
    double kitchenMin = 30;
    double kitchenMax = 1800;
    double toiletMu = 4.8;
    double toiletSigma = 0.7;
    double otherMin = 60;
    double otherMax = 3600;
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
    std::string birthDate;
    std::string sex;
    std::string workplace;
    std::string color;
    bool busy = false;
    int arrivalTime{};
    int departureTime{};
    std::vector<std::string> roomLabels;
    std::vector<std::vector<double>> transitionMatrix;
    StayDurationConfig stayDuration;

    double getStayDuration(const std::string& roomName, std::mt19937& rng) const {
        RoomType roomType = parseRoomName(roomName);
        switch (roomType) {
            case RoomType::WORKPLACE:
                return rnd::uni(rng, stayDuration.workplaceMin, stayDuration.workplaceMax);
            case RoomType::TOILET:
                return rnd::logNormal(rng, stayDuration.toiletMu, stayDuration.toiletSigma);
            case RoomType::KITCHEN:
                return rnd::uni(rng, stayDuration.kitchenMin, stayDuration.kitchenMax);
            case RoomType::OTHER:
                return rnd::uni(rng, stayDuration.otherMin, stayDuration.otherMax);
        }
        return rnd::uni(rng, stayDuration.otherMin, stayDuration.otherMax);
    }

    friend std::ostream& operator<<(std::ostream& os, const Person& p) {
        os << "-------------------------------------------\n"
            << "ID: " << p.id << " | Name: " << p.firstName << " " << p.lastName << "\n"
            << "b-day: " << p.birthDate << " | sex: " << p.sex << "\n"
            << "workplace: " << p.workplace << "\n"
            << "Transition Matrix (Labels: ";

        for (size_t i = 0; i < p.roomLabels.size(); ++i) {
            os << p.roomLabels[i] << (i == p.roomLabels.size() - 1 ? "" : ", ");
        }
        os << "):\n";

        for (const auto& row : p.transitionMatrix) {
            os << "  [ ";
            for (double val : row) {
                os << std::fixed << std::setprecision(2) << val << " ";
            }
            os << "]\n";
        }
        return os;
    }
};

}  // namespace des
