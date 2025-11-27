#pragma once

#include <QObject>
#include <qobject.h>
#include <qobjectdefs.h>

#include "../observer/observer.h"

class ObserverBridge : public QObject , public IObserver {
    Q_OBJECT

public:
    explicit ObserverBridge(QObject* parent = nullptr): QObject(parent) {}

    void onLog(int time, const std::string& message) override {
        emit logReceived(time, QString::fromStdString(message));
    }

    void onRobotMoved(int time, const std::string& location) override {
        emit moveReceived(time, QString::fromStdString(location));
    };

signals:
    void logReceived(int time, QString message);
    void moveReceived(int time, QString location);
};
