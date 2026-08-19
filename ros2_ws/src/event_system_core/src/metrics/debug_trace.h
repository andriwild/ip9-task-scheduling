/*
 * Raw dumps for the debug viewer. Four files, all keyed by simulation time so
 * they can be joined. Nothing derived, each writer is one loop over data that
 * already exists when the simulation ends.
 *
 */

#pragma once

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include "engine/contracts/i_event.h"
#include "engine/event/person_transition.h"
#include "engine/event/stop_drive_event.h"
#include "model/robot.h"
#include "plugins/accompany/events/person_accompany_event.h"
#include "util/point.h"

namespace des::metrics {

inline std::ofstream openTrace(const std::string& path) {
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    return std::ofstream(path);
}

// OUTDOOR and IN_TRANSIT are no rooms and have no coordinates. The line is
// written anyway so the viewer sees that somebody left or is on the way.
inline void writeMoveTrace(const std::string& path, const EventList& protocol) {
    std::ofstream out = openTrace(path);
    out << "t,mission,actor,room,x,y\n";

    auto line = [&out](const IEvent& event, const std::string& actor,
                       const std::string& room, const std::optional<Point>& at) {
        out << event.time << "," << event.getMissionId() << "," << actor << "," << room << ",";
        if (at) {
            out << at->m_x << "," << at->m_y;
        }
        out << "\n";
    };

    for (const auto& event : protocol) {
        if (const auto* drive = dynamic_cast<const StopDriveEvent*>(event.get())) {
            if (drive->m_at) {
                line(*event, "robot", drive->m_room, drive->m_at);
            }
        } else if (const auto* walk = dynamic_cast<const PersonEvent*>(event.get())) {
            if (!walk->m_room.empty()) {
                line(*event, walk->person->firstName, walk->m_room, walk->m_at);
            }
        } else if (const auto* arrived = dynamic_cast<const PersonAccompanyArrivedEvent*>(event.get())) {
            line(*event, arrived->person->firstName, arrived->arrivalRoom, arrived->m_at);
        }
    }
}

inline void writeEventTrace(const std::string& path, const EventList& protocol) {
    std::ofstream out = openTrace(path);
    out << "t,mission,type,name,order,order_state\n";
    for (const auto& event : protocol) {
        const auto order = event->getOrder();
        out << event->time
            << "," << event->getMissionId()
            << "," << static_cast<int>(event->getType())
            << ",\"" << event->getName() << "\""
            << "," << (order ? order->type : "")
            << "," << (order ? static_cast<int>(order->state) : -1)
            << "\n";
    }
}

inline void writeStateTrace(const std::string& path, const StateLog& states) {
    std::ofstream out = openTrace(path);
    out << "t,end,category,name,soc_from,soc_to,dist_from,dist_to\n";
    for (const StateInterval& s : states.entries()) {
        out << s.time << "," << s.endTime
            << "," << static_cast<int>(s.category) << "," << s.name
            << "," << s.socFrom << "," << s.socTo
            << "," << s.distFrom << "," << s.distTo << "\n";
    }
}

inline void writeSightingTrace(const std::string& path, const SightingLog& sightings) {
    std::ofstream out = openTrace(path);
    out << "t,person,room,kind\n";
    for (const Sighting& s : sightings.entries()) {
        out << s.time << "," << s.personName << "," << s.location
            << "," << (s.kind == SightingKind::PRESENT ? "PRESENT" : "ABSENT") << "\n";
    }
}

// What the robot remembers per person and room. The truth to compare it against
// sits in moves.csv, so nothing is derived here.
inline void writeKnowledgeTrace(const std::string& path, const SightingLog& sightings) {
    std::ofstream out = openTrace(path);
    out << "person,room,hits,misses\n";
    for (const auto& [person, rooms] : sightings.allCounts()) {
        for (const auto& [room, counts] : rooms) {
            out << person << "," << room << "," << counts.hits << "," << counts.misses << "\n";
        }
    }
}

}  // namespace des::metrics
