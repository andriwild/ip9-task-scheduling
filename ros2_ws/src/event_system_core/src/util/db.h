#pragma once
#include <iostream>
#include <optional>
#include <utility>
#include <QSqlQuery>
#include <QVariant>
#include <QSql>
#include <optional>

#include "types.h"


class DBClient {
    QSqlDatabase m_db;
    std::string m_user;
    std::string m_pw;
    const std::string m_dbName = "wsr" ;
    const std::string m_host = "localhost";
    const int m_port = 5432;

public:
    explicit DBClient(std::string user, std::string pw)
    : m_user(std::move(user)), m_pw(std::move(pw)) {
       init();
    }

    ~DBClient() {
        m_db.close();
    }

    void setCredentials(const std::string& user, const std::string& pw) {
        m_user = user;
        m_pw = pw;
    }

    bool init() {
        m_db = QSqlDatabase::addDatabase("QPSQL");
        m_db.setPort(5432);
        m_db.setHostName(m_host.data());
        m_db.setDatabaseName(m_dbName.data());
        m_db.setUserName(m_user.data());
        m_db.setPassword(m_pw.data());

        if (!m_db.open()) {
            std::cout << "cant open db connection to "<< m_dbName << " [ " << m_user << ", " << m_pw  << " ]"<< std::endl;
            return false;
        }
        return true;
    }

    std::optional<std::vector<des::Point>> entrances() {
        if (!m_db.open()) {
            std::cerr << "Database not connected" << std::endl;
            return std::nullopt;
        }
        std::vector<des::Point> points = {};

        QSqlQuery query("SELECT ST_AsText(coord) FROM room_entrance");

        while (query.next()) {
            QString wkt = query.value(0).toString();
            wkt.remove("POINT Z (").remove("POINT(").remove(")");
            QStringList coords = wkt.split(" ");
            points.emplace_back( coords[0].toDouble(), coords[1].toDouble() );
        }
        return { points };
    }


    std::optional<des::Person> personByName(const std::string& firstName, const std::string& lastName) {
        if (!m_db.open()) {
            std::cerr << "Database not connected" << std::endl;
            return std::nullopt;
        }
        std::cout << "personByName" << std::endl;
        QSqlQuery query;
        query.prepare("SELECT * FROM people WHERE first_name = :firstName AND last_name = :lastName");
        query.bindValue(":firstName", QString::fromStdString(firstName));
        query.bindValue(":lastName", QString::fromStdString(lastName));

        if (!query.exec()) {
            std::cerr << "personByName Query error: "<< firstName << " "<< lastName << std::endl;
            return std::nullopt;
        }

        if(query.next()) {
            des::Person person;
            person.id = query.value("id").toInt();
            person.firstName = query.value("first_name").toString().toStdString();
            person.lastName = query.value("last_name").toString().toStdString();
            person.birthDate = query.value("birth_date").toString().toStdString();
            person.sex = query.value("sex").toString().toStdString();
            person.assignedRoomId = query.value("assigned_room_id").toInt();
            return person;
        }
        return std::nullopt;
    }
};
