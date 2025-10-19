#pragma once
#include <iostream>
#include <optional>
#include <utility>
#include <QSqlQuery>
#include <QVariant>
#include <QSql>

#include "../model/event.h"
#include "log.h"


class DBClient {
    QSqlDatabase m_db;
    std::string m_user;
    std::string m_pw;
    const std::string m_dbName = "wsr" ;
    const std::string m_host = "localhost";
    const int m_port = 5432;

public:
    explicit DBClient(std::string user, std::string pw)
    : m_user(std::move(user)), m_pw(std::move(pw)) { }

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
            std::cout << "cant open db connection" << std::endl;
            return false;
        }
        return true;
    }

    std::optional<std::vector<Node>> entrances() {
        if (!m_db.open()) {
            return std::nullopt;
        }
        std::vector<Node> points = {};

        QSqlQuery query("SELECT ST_AsText(coord) FROM room_entrance");

        while (query.next()) {
            QString wkt = query.value(0).toString();
            wkt.remove("POINT Z (").remove("POINT(").remove(")");
            QStringList coords = wkt.split(" ");
            points.push_back({ coords[0].toDouble(), coords[1].toDouble() });
        }
        return { points };
    }
};