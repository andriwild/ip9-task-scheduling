#pragma once

#include <memory>
#include <iostream>
#include <vector>
#include <queue>
#include "../util/types.h"


class SimulationContext;

class IEvent {
public:
    int time;
    IEvent(const int time): time(time) {}

    virtual ~IEvent() = default;

    virtual void execute(SimulationContext& ctx) = 0; 
    virtual std::string getName() const = 0; 

    bool operator<(const IEvent& other) const {
        return time < other.time;
    }

    friend std::ostream& operator<<(std::ostream& os, const IEvent& event) {
        os << "[" << event.time << "] " << event.getName();
        return os;
    }
};


struct EventComparator {
    bool operator()(const std::shared_ptr<IEvent>& a, const std::shared_ptr<IEvent>& b) {
        if(!a || !b) return false;
        return a->time > b->time; 
    }
};

using EventQueue  = std::priority_queue<
    std::shared_ptr<IEvent>,
    std::vector<std::shared_ptr<IEvent>>,
    EventComparator
>;

class SimulationStartEvent : public IEvent {
public:
    explicit SimulationStartEvent(int time): IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Sim start"; };

};

class SimulationEndEvent : public IEvent {
public:
    explicit SimulationEndEvent(int time): IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "Sim end"; };
};

class ArrivedEvent : public IEvent {
public:
    std::string location;
    explicit ArrivedEvent(int time, std::string& location): 
        IEvent(time),
        location(location)
    {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "ArrivedEvent"; };
};

class MissionDispatchEvent : public IEvent {
public:
    des::Appointment appointment;
    explicit MissionDispatchEvent(int time, des::Appointment appt): 
        IEvent(time),
        appointment(appt)
    {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "MissionDispatchEvent"; };
};

class MissionCompleteEvent : public IEvent {
public:
    explicit MissionCompleteEvent(int time): IEvent(time) {}
    void execute(SimulationContext& ctx) override;
    std::string getName() const override { return "MissionCompleteEvent"; };
};


// enum class EventType {
//     // --- System & Simulation ---
//     SIMULATION_START,       // Initialisierung (t=0)
//     SIMULATION_END,         // Ende des Tages (t=24h)
    
//     // --- Der "Trigger" aus dem Therapieplan ---
//     MISSION_DISPATCH,       // Ein Termin steht an (Zeitpunkt: Termin - Puffer)

//     // --- Navigation (Basis-Bewegung) ---
//     MOVE_ARRIVED,           // Roboter erreicht einen Knoten (Raum/Kreuzung)
    
//     // --- Suche & Interaktion (Die "Mission") ---
//     SCAN_FINISHED,          // 360° Kamera-Scan im Raum abgeschlossen
//     INTERACTION_FINISHED,   // Gespräch mit Person beendet (Erfolg/Misserfolg)
//     WAIT_FINISHED,          // Warten beendet (wenn man zu früh beim Termin war)
    
//     // --- Eskorte (Spezielle Events während der Begleitung) ---
//     ESCORT_CHECK,           // Periodischer Check: Ist die Person noch hinter mir?
    
//     // --- Sekundäraufgaben (Datenerfassung) ---
//     DATA_TASK_START,        // Startet eine Datenerfassung (wenn IDLE)
//     DATA_READING_FINISHED,  // Schild lesen abgeschlossen
    
//     // --- Energie & Maintenance ---
//     BATTERY_LEVEL_CHECK,    // (Optional) Periodischer Check
//     CHARGING_FINISHED       // Batterie ist wieder voll
// };
