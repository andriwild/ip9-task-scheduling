#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "algo/background/op_solver.h"
#include "op_svg.h"

using des::OpNode;
using des::OpParams;
using des::OpInstance;
using json = nlohmann::json;

namespace {

const std::string BUILDING = "../config/building.json";
const std::string SIMCFG   = "../config/default/sim_config.json";

const std::vector<std::string> KANDIDATEN = {
    "5.2A01", "5.2A07", "5.2A12", "5.2A71", "5.2A15", "5.2A17",
    "5.2D02", "5.2D11", "5.2C59", "5.2C08", "5.2C54", "5.2C91",
    "5.2B53", "5.2B51", "IMVS_Kitchen", "5.2B31", "5.2B13", "5.2B17",
};
const std::vector<char> SEITEN = {
    'l', 'l', 'r',                          // Start, Termin, Dock
    'r', 'l', 'r', 'd', 'r', 'r',           // A01 A07 A12 A71 A15 A17
    'd', 'r', 'd', 'r', 'r', 'r',           // D02 D11 C59 C08 C54 C91
    'l', 'l', 'l', 'r', 'r', 'r',           // B53 B51 Küche B31 B13 B17
};

const std::string START_RAUM = "5.2A05";        // Standort des Roboters
const std::string DOCK_RAUM  = "IMVS_Dock";     // sim_config.dock_location
const std::string END_RAUM   = "IMVS_Printer";  // Ort der Termine aus appointments.json

std::string anzeige(const std::string& name) {
    if (name == "IMVS_Kitchen") { return "Küche"; }
    if (name == "IMVS_Dock")    { return "Dock"; }
    if (name == "IMVS_Printer") { return "Termin"; }
    if (name == START_RAUM)     { return "Start"; }
    return name;
}

struct Variante {
    float timeBudget;
    std::string datei;
    std::string titel;
};

const std::vector<Variante> VARIANTEN = {
    { 1200.0f,  "op-route-kurz.svg", "knappes Zeitbudget"   },
    { 3600.0f,  "op-route.svg",      "mittleres Zeitbudget" },
    { 14400.0f, "op-route-lang.svg", "weites Zeitbudget"    },
};

json lies(const std::string& pfad) {
    std::ifstream f(pfad);
    if (!f) { throw std::runtime_error("nicht lesbar: " + pfad); }
    json j;
    f >> j;
    return j;
}

Point schwerpunkt(const json& raum) {
    if (!raum.contains("footprint")) {
        return { raum.at("x").get<float>(), raum.at("y").get<float>() };
    }
    float sx = 0.0f, sy = 0.0f;
    const auto& fp = raum.at("footprint");
    for (const auto& p : fp) {
        sx += p[0].get<float>();
        sy += p[1].get<float>();
    }
    return { sx / fp.size(), sy / fp.size() };
}

}  // namespace

int main() {
    const json gebaeude = lies(BUILDING);
    const json cfg      = lies(SIMCFG);

    // Index der Raeume im Gebaeudemodell, fuer den Zugriff auf die Distanzmatrix
    std::vector<std::string> alleNamen;
    for (const auto& r : gebaeude.at("rooms")) {
        alleNamen.push_back(r.at("name").get<std::string>());
    }
    auto indexVon = [&](const std::string& name) {
        const auto it = std::find(alleNamen.begin(), alleNamen.end(), name);
        if (it == alleNamen.end()) { throw std::runtime_error("unbekannter Raum: " + name); }
        return static_cast<std::size_t>(std::distance(alleNamen.begin(), it));
    };

    // Werte aus der Konfiguration
    const double rewardWeight   = cfg.at("clean").at("reward_weight").get<double>();
    const double cleaningArea   = cfg.at("clean").at("cleaning_area").get<double>();
    const double cleaningPower  = cfg.at("clean").at("cleaning_power").get<double>();
    const double robotSpeed     = cfg.at("robot_speed").get<double>();
    const double driveW         = cfg.at("energy_consumption_drive").get<double>();
    const double baseW          = cfg.at("energy_consumption_base").get<double>();
    const double chargingRate   = cfg.at("charging_rate").get<double>();
    const double taperFraction  = cfg.at("taper_fraction").get<double>();
    const double cvThreshold    = cfg.at("cv_threshold").get<double>();
    const double lowThreshold   = cfg.at("low_battery_threshold").get<double>();
    const double fullThreshold  = cfg.at("full_battery_threshold").get<double>();
    const double capacityWh     = cfg.at("battery_capacity").get<double>()
                                * cfg.at("battery_voltage").get<double>();

    std::vector<std::string> modellNamen = { START_RAUM, END_RAUM, DOCK_RAUM };
    modellNamen.insert(modellNamen.end(), KANDIDATEN.begin(), KANDIDATEN.end());

    std::vector<OpNode> nodes;
    std::vector<Point> pos;
    for (const auto& name : modellNamen) {
        const auto& raum = gebaeude.at("rooms").at(indexVon(name));
        pos.push_back(schwerpunkt(raum));

        const bool kandidat = std::find(KANDIDATEN.begin(), KANDIDATEN.end(), name)
                              != KANDIDATEN.end();
        if (!kandidat) {
            nodes.push_back({ anzeige(name), 0.0f, 0.0f, 0.0f });
            continue;
        }
        const double flaeche = raum.value("area", 0.0);
        const double reward  = rewardWeight * std::min(1.0, flaeche / 100.0);
        const double dauer   = (flaeche / cleaningArea + 1.0)
                             * (2.0 * std::sqrt(cleaningArea) / robotSpeed);
        nodes.push_back({ anzeige(name),
                          static_cast<float>(reward),
                          static_cast<float>(dauer),
                          static_cast<float>(dauer * cleaningPower / 3600.0) });
    }

    const auto& mat = gebaeude.at("mat");
    const std::size_t n = nodes.size();
    des::Mat distances(n, std::vector<float>(n, 0.0f));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j) { continue; }
            distances[i][j] = mat.at(indexVon(modellNamen[i]))
                                 .at(indexVon(modellNamen[j])).get<float>();
        }
    }

    const std::vector<int> stations = { 2 };  // Dock

    const double netChargeW = chargingRate - baseW;
    OpParams params;
    params.startNodeId  = 0;
    params.endNodeId    = 1;
    params.driveSpeed   = static_cast<float>(robotSpeed);
    params.driveEnergy  = static_cast<float>(driveW / (3600.0 * robotSpeed));
    params.initialSoc   = static_cast<float>(0.45 * capacityWh);
    params.endSocMin    = static_cast<float>(0.20 * capacityWh);
    params.energyBudget = params.initialSoc - params.endSocMin;
    params.maxEnergy    = static_cast<float>(0.50 * capacityWh);
    params.socThreshold = static_cast<float>(lowThreshold / 100.0 * capacityWh);
    params.chargeTimePerWh        = static_cast<float>(3600.0 / netChargeW);
    params.chargeTimePerWhTapered = static_cast<float>(3600.0 / (netChargeW * taperFraction));
    params.cvEnergy               = static_cast<float>(cvThreshold * capacityWh);
    params.costAware              = true;
    params.openEnd                = false;

    std::cout << "Batterie " << capacityWh << " Wh, Start bei " << params.initialSoc
              << " Wh, Reserve " << params.endSocMin << " Wh, Energiebudget "
              << params.energyBudget << " Wh\n\n";

    for (const auto& v : VARIANTEN) {
        params.timeBudget = v.timeBudget;
        const OpInstance op(nodes, distances, stations, params);
        const std::vector<int> route = des::op_solver::grasp(op, 200, 0.3f, 42);
        const auto sim = op.simulateRoute(route, true);

        int raeume = 0;
        for (const int idx : route) {
            if (nodes[idx].reward > 0.0f) { ++raeume; }
        }

        std::cout << v.titel << "  (T_max = " << v.timeBudget / 60.0f << " min)\n";
        std::cout << "  Route: " << nodes[params.startNodeId].name;
        for (const int idx : route) { std::cout << " -> " << nodes[idx].name; }
        std::cout << " -> " << nodes[params.endNodeId].name << "\n";
        std::cout << "  Räume " << raeume << " von " << KANDIDATEN.size()
                  << ", Nutzen " << op.routeReward(route)
                  << ", Zeit " << sim.time / 60.0f << " min"
                  << ", Ladung am Ende " << sim.socEnd << " Wh"
                  << ", verbraucht " << params.initialSoc - sim.socEnd << " Wh"
                  << ", machbar " << sim.feasible << "\n\n";

        writeSvg(nodes, pos, route, params, stations, v.datei, SEITEN);
    }
    return 0;
}
