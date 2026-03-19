#pragma once
#include <QSql>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "types.h"

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
    const std::string m_dbName = "wsr";
    const std::string m_host = "localhost";
    const int m_port = 5432;

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

    void setCredentials(const std::string& user, const std::string& pw) {
        m_user = user;
        m_pw = pw;
    }

    bool init() {
        m_db = QSqlDatabase::addDatabase("QPSQL");
        m_db.setPort(m_port);
        m_db.setHostName(m_host.data());
        m_db.setDatabaseName(m_dbName.data());
        m_db.setUserName(m_user.data());
        m_db.setPassword(m_pw.data());

        if (!m_db.open()) {
            RCLCPP_ERROR(rclcpp::get_logger("DBClient"), "cant open db connection to %s [ %s, %s ]", m_dbName.c_str(), m_user.c_str(), m_pw.c_str());
            return false;
        }
        return true;
    }

    std::optional<std::vector<des::Location>> waypoints() {
        if (!m_db.isOpen() && !m_db.open()) {
            RCLCPP_ERROR(rclcpp::get_logger("DBClient"), "Database error: %s", m_db.lastError().text().toStdString().c_str());
            return std::nullopt;
        }
        std::vector<des::Location> locations;
        QSqlQuery query;
        if (!query.exec("SELECT name, ST_X(coordinate), ST_Y(coordinate), yaw FROM points_of_interest")) {
            RCLCPP_ERROR(rclcpp::get_logger("DBClient"), "Query failed: %s", query.lastError().text().toStdString().c_str());
            return std::nullopt;
        }
        while (query.next()) {
            QString name = query.value(0).toString();
            double x = query.value(1).toDouble();
            double y = query.value(2).toDouble();
            double yaw = query.value(3).toDouble();
            des::Point p = {x, y, yaw};
            locations.emplace_back(name.toStdString(), p);
        }

        return locations;
    }

    std::optional<des::Person> personByName(const std::string& firstName, const std::string& lastName) {
        if (!m_db.open()) {
            RCLCPP_ERROR(rclcpp::get_logger("DBClient"), "Database not connected");
            return std::nullopt;
        }
        RCLCPP_INFO(rclcpp::get_logger("DBClient"), "personByName");
        QSqlQuery query;
        query.prepare("SELECT * FROM people WHERE first_name = :firstName AND last_name = :lastName");
        query.bindValue(":firstName", QString::fromStdString(firstName));
        query.bindValue(":lastName", QString::fromStdString(lastName));

        if (!query.exec()) {
            RCLCPP_ERROR(rclcpp::get_logger("DBClient"), "personByName Query error: %s %s", firstName.c_str(), lastName.c_str());
            return std::nullopt;
        }

        if (query.next()) {
            des::Person person;
            person.id             = query.value("id").toInt();
            person.firstName      = query.value("first_name").toString().toStdString();
            person.lastName       = query.value("last_name").toString().toStdString();
            person.birthDate      = query.value("birth_date").toString().toStdString();
            person.sex            = query.value("sex").toString().toStdString();
            person.workplace = query.value("assigned_room_id").toInt();
            return person;
        }
        return std::nullopt;
    }
};
