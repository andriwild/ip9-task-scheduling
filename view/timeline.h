#pragma once

#include <qgraphicsitem.h>
#include <QGraphicsView>
#include <qobjectdefs.h>
#include <iostream>

#include "../util/types.h"

constexpr int TIMELINE_HEIGHT = 700;
constexpr int TIMELINE_SCENE_MARGIN = 50;
constexpr int Y_LINE_POS = 600;
constexpr int Z_TIMELINE = 100;
constexpr int LABEL_OFFSET = 20;
constexpr int X_LINE_START = TIMELINE_SCENE_MARGIN;

class Timeline final : public QGraphicsView {
    Q_OBJECT

    QGraphicsScene* m_scene;
    const int m_yLinePos = Y_LINE_POS;
    const int m_tickStep = 10;
    const int m_endTime;
    const int m_startTime;
    int m_duration;

public:
    explicit Timeline(int start, int end): 
        QGraphicsView(),
        m_startTime(start), 
        m_endTime(end) 
    {
        m_duration = m_endTime - m_startTime;
        m_scene = new QGraphicsScene(this);
        m_scene->setSceneRect(-TIMELINE_SCENE_MARGIN, 0, m_duration + TIMELINE_SCENE_MARGIN, TIMELINE_HEIGHT);

        setDragMode(ScrollHandDrag);
        setRenderHint(QPainter::Antialiasing);
        setScene(m_scene);
        centerOn(0,0);

        drawBaseline();
        drawTicks();
    }

    void drawDailyPlan(std::vector<des::Appointment> dailyPlan) {
        for(auto appt: dailyPlan){
            auto xPos = appt.appointmentTime - m_startTime + X_LINE_START;
            auto line = m_scene->addLine(
                xPos,
                Y_LINE_POS + 10,
                xPos,
                Y_LINE_POS - 20, 
                { Qt::red, 3 }
            );
        }
    }

public slots:
    void handleLog(int time, QString message) {
        std::cout << "GUI Log: " << message.toStdString() << std::endl;
    }

    void handleMove(int time, QString location) {
        std::cout << "GUI Log (location): " << location.toStdString() << std::endl;
        auto line = m_scene->addLine(
            time - m_startTime,
            Y_LINE_POS + 10,
            time - m_startTime,
            Y_LINE_POS - 20, 
            { Qt::blue, 2 }
        );
    }


private:
    void drawBaseline() const {
        // baseline of timeline (x-axis)
        QGraphicsLineItem* const line  = m_scene->addLine(
            X_LINE_START,
            m_yLinePos,
            m_duration - m_tickStep + X_LINE_START,
            m_yLinePos,
            {Qt::black, 2}
            );
        line->setZValue(Z_TIMELINE);
    }

    void drawTicks() const {
        int time = m_startTime;
        for (int xPosition = X_LINE_START; xPosition < m_duration; xPosition += m_tickStep) {
            if (time % 100 == 0) {
                constexpr int tickHeight = 5;
                QGraphicsLineItem* const line = m_scene->addLine(
                    xPosition,
                    m_yLinePos + tickHeight,
                    xPosition,
                    m_yLinePos - tickHeight,
                    {Qt::black, 2}
                    );
                line->setZValue(Z_TIMELINE);
                QGraphicsSimpleTextItem* const tickLabel = m_scene->addSimpleText(QString::number(time));
                const double w = tickLabel->boundingRect().width();
                tickLabel->setPos(xPosition - w / 2.0, m_yLinePos + LABEL_OFFSET);
            } else {
                constexpr int smallTickHeight = 3;
                QGraphicsLineItem* const line =  m_scene->addLine(
                    xPosition,
                    m_yLinePos + smallTickHeight,
                    xPosition,
                    m_yLinePos - smallTickHeight,
                    {Qt::black, 1}
                    );
                line->setZValue(Z_TIMELINE);
            }
            time += m_tickStep;
        }
    }

};
