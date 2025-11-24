#pragma once

#include "../datastructure/tree.h"
#include "../model/event.h"

auto shiftTime = [](IEvent& ev, const int time){ ev.setTime(ev.getTime() + time);};

class EventTreeComposer {
    EV::Tree<IEvent> tree;
    EV::TreeNode<IEvent>* m_tree;
public:

    EventTreeComposer* fromNewTree() {
        m_tree = tree.createRoot(std::make_unique<Tour>(0, ""));
        return this;
    }
    EV::Tree<IEvent> build() {
        return std::move(tree);
    }

    EventTreeComposer* on(EV::TreeNode<IEvent>  *tree) {
        m_tree = tree;
        return this;
    }

    EventTreeComposer* prepend(EV::Tree<IEvent> & subtree) {
        int mainEventTime = m_tree->getLeftMostLeaf()->getValue()->getTime();
        auto subTreeDuration = subtree.getRightMostLeaf()->getValue()->getTime();
        auto n = subtree.releaseRoot();
        n->forEachSorted(shiftTime, mainEventTime - subTreeDuration);
        m_tree->addSubtree(std::move(n));
        return this;
    }

    EventTreeComposer* append(EV::Tree<IEvent> & subtree) {
        int t = m_tree->getRightMostLeaf()->getValue()->getTime();
        auto n = subtree.releaseRoot();
        n->forEachSorted(shiftTime, t);
        m_tree->addSubtree(std::move(n));
        return this;
    }
};
