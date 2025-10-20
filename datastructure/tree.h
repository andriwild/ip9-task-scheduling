#pragma once
#include <memory>
#include <vector>

class Obj {
public:
    Obj() = default;
};

template<typename T>
class TreeNode {
    T data;
    TreeNode *parent;
    std::vector<std::unique_ptr<TreeNode<T>>> children;
public:

    TreeNode() : data(), parent(nullptr) {}

    TreeNode(T data, TreeNode *parent):
    data(std::move(data)),
    parent(parent)
    {}

    void addChild(T t) {
       auto p = std::make_unique<TreeNode<T>>(t);
       children.push_back(std::move(p));
    }
};


template<typename T>
class Tree {

public:
    Tree()  = default;

    TreeNode<T>* createRoot(T t) {
        // TreeNode<T> node(t, nullptr);
        // m_root = std::make_unique<TreeNode<T>>(std::move(node));

        return m_root.get();
    }

private:
    std::unique_ptr<TreeNode<T>> m_root;
};
