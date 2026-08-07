#pragma once

#include <QSql>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "../model/person.h"
#include "../model/room.h"
#include "../util/log.h"

namespace des {

struct DBConfig {
    std::string m_user;
    std::string m_pw;
    const std::string m_dbName = "wsr";
    const std::string m_host = "localhost";
    const int m_port = 5432;
};

class DBClient {
    QSqlDatabase m_db;
    std::string m_user;
    std::string m_pw;
    const std::string m_dbName;
    const std::string m_host;
    const int m_port;

public:
    explicit DBClient(const DBConfig& dbCfg)
    : m_user(dbCfg.m_user)
    , m_pw(dbCfg.m_pw)
    , m_dbName(dbCfg.m_dbName)
    , m_host(dbCfg.m_host)
    , m_port((dbCfg.m_port))
    {
        init();
    }

    ~DBClient() { m_db.close(); }

    bool init() {
        m_db = QSqlDatabase::addDatabase("QPSQL");
        m_db.setPort(m_port);
        m_db.setHostName(m_host.data());
        m_db.setDatabaseName(m_dbName.data());
        m_db.setUserName(m_user.data());
        m_db.setPassword(m_pw.data());

        if (!m_db.open()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.db"), "cant open db connection to %s [ %s, %s ]", m_dbName.c_str(), m_user.c_str(), m_pw.c_str());
            return false;
        }
        return true;
    }

    std::optional<Person> personByName(const std::string& firstName, const std::string& lastName) {
        if (!m_db.open()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.db"), "Database not connected");
            return std::nullopt;
        }
        DES_LOG_DEBUG(rclcpp::get_logger("des.io.db"), "personByName");
        QSqlQuery query;
        query.prepare("SELECT * FROM people WHERE first_name = :firstName AND last_name = :lastName");
        query.bindValue(":firstName", QString::fromStdString(firstName));
        query.bindValue(":lastName", QString::fromStdString(lastName));

        if (!query.exec()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.db"), "personByName Query error: %s %s", firstName.c_str(), lastName.c_str());
            return std::nullopt;
        }

        if (query.next()) {
            Person person;
            person.id        = query.value("id").toInt();
            person.firstName = query.value("first_name").toString().toStdString();
            person.lastName  = query.value("last_name").toString().toStdString();
            person.sex       = query.value("sex").toString().toStdString();
            person.workplace = query.value("assigned_room_id").toInt();
            return person;
        }
        return std::nullopt;

    }

    std::optional<double> areaByName(const std::string& zoneName) {
        if (!m_db.open()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.db"), "Database not connected");
            return std::nullopt;
        }
        QSqlQuery query;
        query.prepare("SELECT ST_Area(sz.polygon) FROM search_zones sz JOIN points_of_interest p ON p.id = sz.poi_id WHERE p.name = :zoneName");
        query.bindValue(":zoneName", QString::fromStdString(zoneName));

        if (!query.exec()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.db"), "areaByName Query error: %s", zoneName.c_str());
            return std::nullopt;
        }

        if (query.next()) {
            return query.value(0).toDouble();
        }
        return std::nullopt;
    }

    std::optional<RoomMap> rooms() {
        if (!m_db.isOpen() && !m_db.open()) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.db"), "Database error: %s", m_db.lastError().text().toStdString().c_str());
            return std::nullopt;
        }
        QSqlQuery query;
        if (!query.exec(
                "SELECT p.name, ST_X(p.coordinate), ST_Y(p.coordinate), COALESCE(p.yaw, 0), "
                "COALESCE(sz.type, 'OTHER'), ST_Area(sz.polygon), ST_X(d.geom), ST_Y(d.geom) "
                "FROM points_of_interest p "
                "LEFT JOIN search_zones sz ON sz.poi_id = p.id "
                "LEFT JOIN LATERAL ST_DumpPoints(ST_ExteriorRing(sz.polygon)) d ON true "
                "WHERE d.path IS NULL OR d.path[1] < ST_NPoints(ST_ExteriorRing(sz.polygon)) "
                "ORDER BY p.name, d.path[1]")) {
            DES_LOG_ERROR(rclcpp::get_logger("des.io.db"), "rooms Query failed: %s", query.lastError().text().toStdString().c_str());
            return std::nullopt;
        }
        RoomMap rooms;
        while (query.next()) {
            const std::string name = query.value(0).toString().toStdString();
            const Point p     = {query.value(1).toDouble(), query.value(2).toDouble(), query.value(3).toDouble()};
            const auto [it, inserted] = rooms.try_emplace(name, name, p);
            if (inserted) {
                it->second.m_roomType = roomTypeFromString(query.value(4).toString().toStdString());
                if (!query.value(5).isNull()) {
                    it->second.m_area = query.value(5).toDouble();
                }
            }
            if (!query.value(6).isNull()) {
                it->second.m_footprint.push_back({query.value(6).toDouble(), query.value(7).toDouble(), 0.0});
            }
        }
        return rooms;
    }

};

}  // namespace des
