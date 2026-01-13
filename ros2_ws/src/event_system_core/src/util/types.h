#pragma once

#include <iostream>
#include <string>
#include <iostream>
#include <iomanip>

namespace des {
    
	struct Point {
		double m_x, m_y, m_yaw;

		Point() = default;

		Point(const double pnt, const double pnt1, const double yaw):
		    m_x(pnt),
		    m_y(pnt1),
            m_yaw(yaw)
		{}

		friend std::ostream& operator<<(std::ostream& os, const Point& s) {
			os << "(" << s.m_x << ", " << s.m_y << ", " << s.m_yaw << ")";
			return os;
		}

	};

	struct Segment {
		int id;
		Point m_points[2];

		Segment() = default;

		Segment(const int segment_id, const Point& p1, const Point& p2) :
			id(segment_id),
			m_points{p1, p2}
		{}
		friend std::ostream& operator<<(std::ostream& os, const Segment& s) {
			os << "Segment id=" << s.id
			   << ", p1=" << s.m_points[0] << ", p2=" << s.m_points[1];
			return os;
		}
	};


    struct Location {
        std::string m_name;
        Point m_p;
    
        explicit Location(const std::string& name, const Point p):
            m_name(name),
            m_p(p) 
        {}
    
        friend std::ostream& operator<<(std::ostream& os, const Location& l) {
            os << l.m_name << l.m_p;
            return os;
        }
    };

    struct SimConfig {
        double personFindProbability;
        double robotSpeed;
        double robotEscortSpeed;
        double driveStd;
        double conversationFoundStd;
        double conversationDropOffStd;
        double missionOverhead;
        double timeBuffer;

        friend std::ostream& operator<<(std::ostream& os, const SimConfig& c) {
            os << "............\n";
            os << "personFindProbability: " << c.personFindProbability << "\n";
            os << "robotSpeed: " << c.robotSpeed << "\n";
            os << "robotEscortSpeed: " << c.robotEscortSpeed << "\n";
            os << "driveStd: " << c.driveStd << "\n";
            os << "conversationFoundStd: " << c.conversationFoundStd << "\n";
            os << "conversationDropOffStd: " << c.conversationDropOffStd << "\n";
            os << "missionOverhead: " << c.missionOverhead << "\n";
            os << "timeBuffer: " << c.timeBuffer << "\n";
            os << "............\n";
            return os;
        }
    };

	struct Person {
		int id;
		std::string firstName;
		std::string lastName;
		std::string birthDate;
		std::string sex;
		int assignedRoomId;
	};

    enum MissionState { 
        PENDING, 
        COMPLETED,
        IN_PROGRESS, 
        FAILED, 
        CANCELLED 
    };

    struct Appointment {
        int id;
        std::string personName;
        std::string roomName;
        int appointmentTime;
        std::string description;
        MissionState state = PENDING;
    };

    inline std::string toHumanReadableTime(const int sec, bool includeSeconds = true) {
        int hours = static_cast<int>(sec/ 3600.0);
        int minutes = static_cast<int>((sec- hours * 3600.0) / 60.0);
        int seconds = static_cast<int>(sec) % 60;
    
        std::ostringstream oss;
        oss << std::setw(2) << std::setfill('0') << hours << ":"
            << std::setw(2) << std::setfill('0') << minutes;
        if (includeSeconds){
            oss << ":" << std::setw(2) << std::setfill('0') << seconds;
        }
        return oss.str();
    }
}

