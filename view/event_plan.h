#pragma once

#include <QGraphicsItemGroup>
#include <QGraphicsRectItem>
#include <QPen>
#include <QBrush>
#include "../datastructure/tree.h"
#include <QColor>

class EventPlan: public QGraphicsItemGroup {
    int m_startTime = 0;
    int m_xOffset;
    int m_yLevel = 0;
    int driveStart = 0;
    int talkStart = 0;
    int LINE_GAP = 20;

public:
    EventPlan(TreeNode<SimulationEvent> &root, const int xOffset, QGraphicsItem* parent = nullptr):
    QGraphicsItemGroup(parent),
    m_xOffset(xOffset)
    {
        drawRec(&root);
    }

    QRectF boundingRect() const override {
        return childrenBoundingRect();
    }


private:
    void drawBlock(const int start, const int end, const int y, QColor color) {
        const int recStart = start + m_xOffset;
        const int recEnd = end + m_xOffset;
        auto r = new QGraphicsRectItem(recStart, y, recEnd - recStart, 10, this);
        r->setPen(QPen(Qt::black, 1));
        r->setBrush(color);
    }

    void draw(TreeNode<SimulationEvent> * node, int level) {
        level = level * LINE_GAP;
        const int s = node->getLeftMostLeaf()->getValue()->getTime();
        const int e = node->getRightMostLeaf()->getValue()->getTime();

        SimulationEvent* currentEvent = node->getValue();
        if (dynamic_cast<MeetingEvent*>(currentEvent)) {
            drawBlock(s, e, level, QColor(0, 140, 140));
        } else if (dynamic_cast<MeetingEvent*>(currentEvent)) {
            drawBlock(s, e, level, QColor(0, 140, 140));
        } else if (dynamic_cast<RobotDriveStartEvent*>(node->getValue())) {
            driveStart = node->getValue()->getTime();
        } else if (dynamic_cast<RobotDriveEndEvent*>(node->getValue())) {
            drawBlock(driveStart, node->getValue()->getTime(), level, QColor(40, 40, 40));
        } else if (dynamic_cast<SearchEvent*>(node->getValue())) {
            drawBlock(s, e, level, QColor(50, 90, 140));
        } else if (dynamic_cast<TalkingEventStart*>(node->getValue())) {
            talkStart = node->getValue()->getTime();
        } else if (dynamic_cast<TalkingEventEnd*>(node->getValue())) {
            drawBlock(talkStart, node->getValue()->getTime(), level, QColor(90, 190, 140));
        } else if (dynamic_cast<Tour*>(node->getValue())) {
            drawBlock(s, e, level, QColor(0, 0, 240));
        } else if (dynamic_cast<EscortEvent*>(node->getValue())) {
            drawBlock(s, e, level, QColor(200, 240, 140));
        }
    }

    void drawRec(TreeNode<SimulationEvent>* node, int level = 0) {
            level ++;
        if(!node->isLeaf()) {
            for (const auto& child: node->getChildren()) {
                drawRec(child.get(), level);
            }
        }
        draw(node, level);
    }
};