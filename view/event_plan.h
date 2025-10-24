#pragma once

#include <QGraphicsItemGroup>
#include <QGraphicsRectItem>
#include <QPen>
#include <QBrush>
#include "../datastructure/tree.h"
#include <QColor>

class EventPlan : public QGraphicsItemGroup {
    int m_startTime = 0;
    int m_xOffset;
    int m_yLevel = 0;
    int driveStart = 0;
    int talkStart = 0;
    int LINE_GAP = 20;

public:
    EventPlan(TreeNode<SimulationEvent> &root, const int xOffset,
              QGraphicsItem *parent = nullptr) : QGraphicsItemGroup(parent),
                                                 m_xOffset(xOffset) {
        drawRec(&root);
    }

    QRectF boundingRect() const override {
        return childrenBoundingRect();
    }

private:
    void drawBlock(const int start, const int end, const int y, QColor color, std::string label = "",
                   int xMarker = -1) {
        const int recStart = start + m_xOffset;
        const int recEnd = end + m_xOffset;
        const int length = recEnd - recStart;
        const int blockHeight = 16;

        auto r = new QGraphicsRectItem(recStart, y, length, blockHeight, this);
        r->setPen({Qt::black, 1});
        r->setBrush(color);

        if (xMarker != -1) {
            auto l = new QGraphicsLineItem(
                xMarker + m_xOffset - 1,
                y + blockHeight -1,
                xMarker + m_xOffset - 1,
                y + 1,
                this
                );
            l->setPen({Qt::red, 2});
        }

        auto stringLabel = new QGraphicsSimpleTextItem(QString::fromStdString(label), this);
        const double w = stringLabel->boundingRect().width();
        QFont font = stringLabel->font();
        font.setBold(false);
        font.setPointSize(8);
        stringLabel->setFont(font);
        stringLabel->setBrush(Qt::black);
        stringLabel->setPos(recStart + length / 2.0 - w / 2.0, y);

        if (w > length) {
            delete stringLabel;
        }
    }

    void draw(TreeNode<SimulationEvent> *node, int level) {
        level = level * LINE_GAP;
        const int s = node->getLeftMostLeaf()->getValue()->getTime();
        const int e = node->getRightMostLeaf()->getValue()->getTime();

        SimulationEvent *ev = node->getValue();
        if (dynamic_cast<MeetingEvent *>(ev)) {
            drawBlock(s, e, level, Helper::color(MEETING), ev->getLabel(), ev->getTime());
        } else if (dynamic_cast<RobotDriveStartEvent *>(ev)) {
            driveStart = ev->getTime();
        } else if (dynamic_cast<RobotDriveEndEvent *>(ev)) {
            drawBlock(driveStart, ev->getTime(), level, Helper::color(DRIVE));
        } else if (dynamic_cast<SearchEvent *>(ev)) {
            drawBlock(s, e, level, Helper::color(SEARCH), ev->getLabel());
        } else if (dynamic_cast<TalkingEventStart *>(ev)) {
            talkStart = ev->getTime();
        } else if (dynamic_cast<TalkingEventEnd *>(ev)) {
            drawBlock(talkStart, ev->getTime(), level, Helper::color(TALK));
        } else if (dynamic_cast<Tour *>(ev)) {
            drawBlock(s, e, level, Helper::color(TOUR), ev->getLabel());
        } else if (dynamic_cast<EscortEvent *>(ev)) {
            drawBlock(s, e, level, Helper::color(ESCORT), ev->getLabel());
        }
    }

    void drawRec(TreeNode<SimulationEvent> *node, int level = 0) {
        level++;
        if (!node->isLeaf()) {
            for (const auto &child: node->getChildren()) {
                drawRec(child.get(), level);
            }
        }
        draw(node, level);
    }
};
