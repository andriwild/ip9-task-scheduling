#pragma once
#include <memory>
#include <vector>

template<typename T>
class TreeNode {
    T data;
    TreeNode *parent;
    std::vector<std::unique_ptr<TreeNode<T>>> children;

public:
    TreeNode(T data, TreeNode *parent):
    data(std::move(data)),
    parent(parent)
    {}

    TreeNode* addChild(T t) {
        auto p = std::make_unique<TreeNode<T>>(std::move(t), this);
        auto* child = p.get();
        children.emplace_back(std::move(p));
        return child;
    }

    std::vector<TreeNode*> getAllDescendants() {
        std::vector<TreeNode*> result;
        for (const auto& child: children) {
            result.emplace_back(child.get());
            auto nodes = child->getAllDescendants();
            result.insert(result.end(), nodes.begin(), nodes.end());
        }
        return result;
    }


    void removeChild(TreeNode* node) {

    }

    bool isLeaf() {
        return children.empty();
    }
};


template<typename T>
class Tree {

public:
    Tree()  = default;

    TreeNode<T>* createRoot(T t) {
        TreeNode<T> node(t, nullptr);
        m_root = std::make_unique<TreeNode<T>>(std::move(node));
        return m_root.get();
    }

    TreeNode<T>* getRoot() {
        return m_root.get();
    }

    std::vector<TreeNode<T>*> getAllNodes() {
        assert(m_root != nullptr);
        std::vector<TreeNode<T>*> result;
        result.emplace_back(m_root.get());
        auto children = m_root.getAllDescendants();
        result.insert(result.end(), children.begin(), children.end());
        return result;
    }

    void removeSubtree(TreeNode<T> *n) {

    }

    void clear() {
        m_root.reset();
    }

    bool isEmpty() {
        m_root == nullptr;
    }

private:
    std::unique_ptr<TreeNode<T>> m_root;
};
