#pragma once

#include "../datastructure/tree.h"
#include "../model/event.h"

auto shiftTime = [](SimulationEvent& ev, const int time){ ev.setTime(ev.getTime() + time);};

class EventTreeComposer {
    EV::Tree<SimulationEvent> tree;
    EV::TreeNode<SimulationEvent>* m_tree;
public:

    EventTreeComposer* fromNewTree() {
        m_tree = tree.createRoot(std::make_unique<Tour>(0, ""));
        return this;
    }
    EV::Tree<SimulationEvent> build() {
        return std::move(tree);
    }

    EventTreeComposer* on(EV::TreeNode<SimulationEvent>  *tree) {
        m_tree = tree;
        return this;
    }

    EventTreeComposer* prepend(EV::Tree<SimulationEvent> & subtree) {
        int mainEventTime = m_tree->getLeftMostLeaf()->getValue()->getTime();
        auto subTreeDuration = subtree.getRightMostLeaf()->getValue()->getTime();
        auto n = subtree.releaseRoot();
        n->forEachSorted(shiftTime, mainEventTime - subTreeDuration);
        m_tree->addSubtree(std::move(n));
        return this;
    }

    EventTreeComposer* append(EV::Tree<SimulationEvent> & subtree) {
        int t = m_tree->getRightMostLeaf()->getValue()->getTime();
        auto n = subtree.releaseRoot();
        n->forEachSorted(shiftTime, t);
        m_tree->addSubtree(std::move(n));
        return this;
    }
};