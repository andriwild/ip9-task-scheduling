#pragma once

#include <iomanip>
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "model/room.h"
#include "util/types.h"

namespace des {

struct SimConfig {
    double robotSpeed;
    double driveDelayMedian = 0.05;
    double driveDelaySigma = 0.4;
    double timeBuffer;
    double energyConsumptionDrive;
    double energyConsumptionBase;
    double batteryCapacity;
    double initialBatteryCapacity;
    double chargingRate;
    double lowBatteryThreshold;
    double fullBatteryThreshold;
    double arrivalMean;
    double arrivalStd;
    double departureMean;
    double departureStd;
    DistributionType arrivalDistribution;
    DistributionType departureDistribution;
    std::string dockLocation;
    bool cacheEnabled;
    std::string appointmentsPath;
    std::string peopleSpawnLocation;
    double personIdentificationRange = 5.0;
    double personRecognitionRange = 5.0;
    double personSpeedMale = 1.41;
    double personSpeedFemale = 1.27;
    bool publishPersonEvents = true;
    int simStartTime = 25200;  // 07:00
    int simDuration  = 43200;
    bool useDistanceMatrix = false;
    double batteryVoltage = 12.0;
    double cvThreshold = 0.8;
    double taperFraction = 0.5;
    bool chargeToFull = true;
    bool alwaysChargeAtDock = false;
    bool metricsCsvExport = true;
    bool debugExport = false;
    bool replanBackgroundOnInterrupt = true;
    double lunchMean = 12.0 * 3600;
    double lunchStd = 1800.0;
    DistributionType lunchDistribution = DistributionType::NORMAL;
    double lunchDurationMean = 2400.0;
    double lunchDurationStd = 600.0;
    std::vector<RoomType> searchExcludedRoomTypes = { RoomType::ACCESS, RoomType::TOILET };
    std::string employeesPath = "";
    SearchRewardStrategy searchRewardStrategy = SearchRewardStrategy::BETA_SMOOTHED;
    SearchRouteStrategy searchRouteStrategy = SearchRouteStrategy::COST_AWARE;
    double searchPriorWeight = 4.0;
    double searchWorkplacePrior = 0.6;
    // for the paper: accompany ends with the find of the person
    bool searchDropOffAtFind = false;
    double searchTrueWorkplaceShare = 0.65;
    std::map<RoomType, double> searchTrueDistribution = {
        { RoomType::OFFICE,  0.14 },
        { RoomType::MEETING, 0.10 },
        { RoomType::KITCHEN, 0.04 },
        { RoomType::SPACE,   0.04 },
        { RoomType::TOILET,  0.03 },
    };
    double personDirectionsProbability = 0.0;
    double personDirectionsWrongProbability = 0.0;
    EnergyReserveStrategy energyReserveStrategy = EnergyReserveStrategy::HORIZON;
    int energyReserveHorizon = 4 * 3600;
    bool backgroundCostAware = true;
    int graspIterations = 200;
    double graspAlpha = 0.3;
    unsigned int seed = 42;
    int rounds = 1;
    RoundMode roundMode = RoundMode::REPLICATION;

    friend std::ostream& operator<<(std::ostream& os, const SimConfig& config) {
        const int W = 30;
        os << "\n"
           << "\033[1m" << "--- Configuration Loaded ---" << "\033[0m" << std::endl;
        os << std::left << std::setw(W) << "robotSpeed" << ": " << config.robotSpeed << std::endl;
        os << std::left << std::setw(W) << "driveDelayMedian" << ": " << config.driveDelayMedian << std::endl;
        os << std::left << std::setw(W) << "driveDelaySigma" << ": " << config.driveDelaySigma << std::endl;
        os << std::left << std::setw(W) << "timeBuffer" << ": " << config.timeBuffer << std::endl;
        os << std::left << std::setw(W) << "energyConsumptionDrive" << ": " << config.energyConsumptionDrive << std::endl;
        os << std::left << std::setw(W) << "energyConsumptionBase" << ": " << config.energyConsumptionBase << std::endl;
        os << std::left << std::setw(W) << "batteryCapacity" << ": " << config.batteryCapacity << std::endl;
        os << std::left << std::setw(W) << "initialBatteryCapacity" << ": " << config.initialBatteryCapacity << std::endl;
        os << std::left << std::setw(W) << "chargingRate" << ": " << config.chargingRate << std::endl;
        os << std::left << std::setw(W) << "lowBatteryThreshold" << ": " << config.lowBatteryThreshold << std::endl;
        os << std::left << std::setw(W) << "fullBatteryThreshold" << ": " << config.fullBatteryThreshold << std::endl;
        os << std::left << std::setw(W) << "arrivalMean" << ": " << config.arrivalMean << std::endl;
        os << std::left << std::setw(W) << "arrivalStd" << ": " << config.arrivalStd << std::endl;
        os << std::left << std::setw(W) << "departureMean" << ": " << config.departureMean << std::endl;
        os << std::left << std::setw(W) << "departureStd" << ": " << config.departureStd << std::endl;
        os << std::left << std::setw(W) << "arrivalDistribution" << ": " << distributionTypeToString(config.arrivalDistribution) << std::endl;
        os << std::left << std::setw(W) << "departureDistribution" << ": " << distributionTypeToString(config.departureDistribution) << std::endl;
        os << std::left << std::setw(W) << "lunchMean" << ": " << config.lunchMean << std::endl;
        os << std::left << std::setw(W) << "lunchStd" << ": " << config.lunchStd << std::endl;
        os << std::left << std::setw(W) << "lunchDistribution" << ": " << distributionTypeToString(config.lunchDistribution) << std::endl;
        os << std::left << std::setw(W) << "lunchDurationMean" << ": " << config.lunchDurationMean << std::endl;
        os << std::left << std::setw(W) << "lunchDurationStd" << ": " << config.lunchDurationStd << std::endl;
        os << std::left << std::setw(W) << "dockPose" << ": " << config.dockLocation<< std::endl;
        os << std::left << std::setw(W) << "cache enabled" << ": " << config.cacheEnabled << std::endl;
        os << std::left << std::setw(W) << "appointmentsPath" << ": " << config.appointmentsPath << std::endl;
        os << std::left << std::setw(W) << "employeesPath" << ": " << config.employeesPath << std::endl;
        os << std::left << std::setw(W) << "searchRewardStrategy" << ": " << searchRewardStrategyToString(config.searchRewardStrategy) << std::endl;
        os << std::left << std::setw(W) << "searchRouteStrategy" << ": " << searchRouteStrategyToString(config.searchRouteStrategy) << std::endl;
        os << std::left << std::setw(W) << "energyReserveStrategy" << ": " << energyReserveStrategyToString(config.energyReserveStrategy) << std::endl;
        os << std::left << std::setw(W) << "energyReserveHorizon" << ": " << config.energyReserveHorizon << std::endl;
        os << std::left << std::setw(W) << "seed" << ": " << config.seed << std::endl;
        os << std::left << std::setw(W) << "rounds" << ": " << config.rounds << std::endl;
        os << std::left << std::setw(W) << "roundMode" << ": " << roundModeToString(config.roundMode) << std::endl;
        os << std::left << std::setw(W) << "peopleSpawnLocation" << ": " << config.peopleSpawnLocation << std::endl;
        os << std::left << std::setw(W) << "personIdentificationRange" << ": " << config.personIdentificationRange << std::endl;
        os << std::left << std::setw(W) << "personRecognitionRange" << ": " << config.personRecognitionRange << std::endl;
        os << std::left << std::setw(W) << "personDirectionsProbability" << ": " << config.personDirectionsProbability << std::endl;
        os << std::left << std::setw(W) << "personDirectionsWrongProbability" << ": " << config.personDirectionsWrongProbability << std::endl;
        os << std::left << std::setw(W) << "personSpeedMale" << ": " << config.personSpeedMale << std::endl;
        os << std::left << std::setw(W) << "personSpeedFemale" << ": " << config.personSpeedFemale << std::endl;
        os << std::left << std::setw(W) << "publishPersonEvents" << ": " << config.publishPersonEvents << std::endl;
        os << std::left << std::setw(W) << "simStartTime" << ": " << config.simStartTime << std::endl;
        os << std::left << std::setw(W) << "simDuration" << ": " << config.simDuration << std::endl;
        os << std::left << std::setw(W) << "useDistanceMatrix" << ": " << config.useDistanceMatrix << std::endl;
        os << std::left << std::setw(W) << "batteryVoltage" << ": " << config.batteryVoltage << std::endl;
        os << std::left << std::setw(W) << "cvThreshold" << ": " << config.cvThreshold << std::endl;
        os << std::left << std::setw(W) << "taperFraction" << ": " << config.taperFraction << std::endl;
        os << std::left << std::setw(W) << "chargeToFull" << ": " << config.chargeToFull << std::endl;
        os << std::left << std::setw(W) << "alwaysChargeAtDock" << ": " << config.alwaysChargeAtDock << std::endl;
        os << std::left << std::setw(W) << "metricsCsvExport" << ": " << config.metricsCsvExport << std::endl;
        os << std::left << std::setw(W) << "replanBackgroundOnInterrupt" << ": " << config.replanBackgroundOnInterrupt << std::endl;
        os << std::left << std::setw(W) << "searchExcludedRoomTypes" << ": ";
        for (size_t i = 0; i < config.searchExcludedRoomTypes.size(); ++i) {
            os << (i ? ", " : "") << roomTypeToString(config.searchExcludedRoomTypes[i]);
        }
        os << std::endl;
        os << "----------------------------\n"
           << std::endl;
        return os;
    }
};

}  // namespace des
