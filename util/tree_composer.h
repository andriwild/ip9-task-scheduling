#pragma once

#include "../datastructure/tree.h"
#include "../model/event.h"

auto shiftTime = [](SimulationEvent& ev, const int time){ ev.setTime(ev.getTime() + time);};

class EventTreeComposer {
    Tree<SimulationEvent> tree;
    TreeNode<SimulationEvent>* m_tree;
public:

    EventTreeComposer* fromNewTree() {
        m_tree = tree.createRoot(std::make_unique<Tour>(0, ""));
        return this;
    }
    Tree<SimulationEvent> build() {
        return std::move(tree);
    }

    EventTreeComposer* on(TreeNode<SimulationEvent>  *tree) {
        m_tree = tree;
        return this;
    }

    EventTreeComposer* prepend(Tree<SimulationEvent> & subtree) {
        int mainEventTime = m_tree->getLeftMostLeaf()->getValue()->getTime();
        auto subTreeDuration = subtree.getRightMostLeaf()->getValue()->getTime();
        auto n = subtree.releaseRoot();
        n->forEachSorted(shiftTime, mainEventTime - subTreeDuration);
        m_tree->addSubtree(std::move(n));
        return this;
    }

    EventTreeComposer* append(Tree<SimulationEvent> & subtree) {
        int t = m_tree->getRightMostLeaf()->getValue()->getTime();
        auto n = subtree.releaseRoot();
        n->forEachSorted(shiftTime, t);
        m_tree->addSubtree(std::move(n));
        return this;
    }
};