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
    void drawBlock(const int start, const int end, const int y, QColor color, std::string label = "") {
        const int recStart = start + m_xOffset;
        const int recEnd = end + m_xOffset;
        const int length = recEnd - recStart;

        auto stringLabel = new QGraphicsSimpleTextItem(QString::fromStdString(label), this);
        const double w = stringLabel->boundingRect().width();
        QFont font = stringLabel->font();
        font.setBold(false);
        font.setPointSize(8);
        stringLabel->setFont(font);
        stringLabel->setBrush(Qt::black);
        stringLabel->setPos(recStart + length / 2.0 - w / 2.0, y );
        stringLabel->setZValue(1000);

        if (w > length) {
            delete stringLabel;
        }

        auto r = new QGraphicsRectItem(recStart, y, length, 15, this);
        r->setPen(QPen(Qt::black, 1));
        r->setBrush(color);
    }

    void draw(TreeNode<SimulationEvent> * node, int level) {
        level = level * LINE_GAP;
        const int s = node->getLeftMostLeaf()->getValue()->getTime();
        const int e = node->getRightMostLeaf()->getValue()->getTime();

        SimulationEvent* currentEvent = node->getValue();
        if (dynamic_cast<MeetingEvent*>(currentEvent)) {
            drawBlock(s, e, level, Helper::color(MEETING), node->getValue()->getLabel());
        } else if (dynamic_cast<RobotDriveStartEvent*>(node->getValue())) {
            driveStart = node->getValue()->getTime();
        } else if (dynamic_cast<RobotDriveEndEvent*>(node->getValue())) {
            drawBlock(driveStart, node->getValue()->getTime(), level, Helper::color(DRIVE));
        } else if (dynamic_cast<SearchEvent*>(node->getValue())) {
            drawBlock(s, e, level, Helper::color(SEARCH), node->getValue()->getLabel());
        } else if (dynamic_cast<TalkingEventStart*>(node->getValue())) {
            talkStart = node->getValue()->getTime();
        } else if (dynamic_cast<TalkingEventEnd*>(node->getValue())) {
            drawBlock(talkStart, node->getValue()->getTime(), level, Helper::color(TALK));
        } else if (dynamic_cast<Tour*>(node->getValue())) {
            drawBlock(s, e, level, Helper::color(TOUR), node->getValue()->getLabel());
        } else if (dynamic_cast<EscortEvent*>(node->getValue())) {
            drawBlock(s, e, level, Helper::color(ESCORT), node->getValue()->getLabel());
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