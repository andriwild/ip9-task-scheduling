#pragma once
#include <memory>
#include <vector>


template<typename T>
class TreeNode {
private:
    const std::unique_ptr<T> data;
    std::vector<std::unique_ptr<TreeNode>> children;

    void sortRec() {
        auto comparator = [](const auto& a, const auto&b) {
            return *a->getValue() < *b->getValue();
        };
        std::sort(children.begin(), children.end(), comparator);
        for (const auto& child: children) {
            child->sortRec();
        }
    }

    void forEach(std::function<void(T&, int)> fn,int time) {
        fn(*this->getValue(), time);

        for (const auto& child: children) {
            child->forEach(fn, time);
        }
    }

public:
    TreeNode *parent;

    explicit TreeNode(std::unique_ptr<T> data, TreeNode *parent = nullptr):
    data(std::move(data)),
    parent(parent)
    {}

    TreeNode* addChild(std::unique_ptr<T> t) {
        auto p = std::make_unique<TreeNode>(std::move(t), this);
        auto* child = p.get();
        children.emplace_back(std::move(p));
        sort();
        return child;
    }

    TreeNode* addSubtree(std::unique_ptr<TreeNode> subtreeRoot) {
        subtreeRoot->parent = this;
        auto oldRoot = subtreeRoot.get();
        children.emplace_back(std::move(subtreeRoot));
        sort();
        return oldRoot;
    }

    std::vector<TreeNode*> getAllDescendants() const {
        std::vector<TreeNode*> result;
        for (const auto& child: children) {
            result.emplace_back(child.get());
            auto nodes = child->getAllDescendants();
            result.insert(result.end(), nodes.begin(), nodes.end());
        }
        return result;
    }

    void removeChild(TreeNode* childToRemove) {
        children.erase(
            std::remove_if(
                children.begin(),
                children.end(),
                [childToRemove](const auto& child) {
                    return child.get() == childToRemove;
                }),
            children.end()
        );
    }

    void sort() {
        // go upwards to root
        TreeNode* p = this;
        while (p->parent != nullptr) {
            p = p->parent;
        }
        p->sortRec();
    }

    void forEachSorted(std::function<void(T&, int)> fn,int time) {
        forEach(fn, time);
        sort();
    }

    TreeNode *getLeaf(const bool left) {
        if (isLeaf()) {
            return this;
        }
        if (!children.empty()) {
            if (left) {
                return children.front()->getLeaf(left);
            }
            return children.back()->getLeaf(left);
        }
        assert(false);
    }

    const std::vector<std::unique_ptr<TreeNode>>& getChildren() const { return children; }

    int childCount() const { return children.size(); }

    T* getValue() const { return data.get(); }

    bool isLeaf() const { return children.empty(); }

    TreeNode *getLeftMostLeaf() { return getLeaf(true); }

    TreeNode *getRightMostLeaf() { return getLeaf(false); }
};


template<typename T>
class Tree {

public:
    Tree()  = default;

    TreeNode<T>* createRoot(std::unique_ptr<T> t) {
        m_root = std::make_unique<TreeNode<T>>(std::move(t));
        return m_root.get();
    }

    TreeNode<T>* getRoot() const { return m_root.get(); }

    std::unique_ptr<TreeNode<T>> getRoot() { return std::move(m_root); }


    std::vector<TreeNode<T>*> getAllNodes() {
        if (!m_root) return {};
        std::vector<TreeNode<T>*> result;
        result.emplace_back(m_root.get());
        auto children = m_root->getAllDescendants();
        result.insert(result.end(), children.begin(), children.end());
        return result;
    }

    void removeSubtree(TreeNode<T> *n) {
        if (m_root and m_root.get() == n) {
            clear();
            return;
        }
        if (n->parent) {
            n->parent->removeChild(n);
        }
    }

    void clear() { m_root.reset(); }

    bool isEmpty() const { return m_root == nullptr; }

    TreeNode<T>* getLeftMostLeaf() const { return m_root->getLeaf(true); }

    TreeNode<T>* getRightMostLeaf() const { return m_root->getLeaf(false); }

    std::vector<TreeNode<T>*> traversalPreOrder() {
        std::vector<TreeNode<T>*> result;
        traversalPreOrderRec(m_root.get(), result);
        return result;
    }

    void printNode(std::ostream& os, const TreeNode<T>* node, const std::string& prefix, bool isLast) const {
        if (!node) return;

        os << prefix;
        os << (isLast ? "└── " : "├── ");
        os << *node->getValue() << "\n";

        const std::string childPrefix = prefix + (isLast ? "    " : "│   ");

        for (size_t i = 0; i < node->childCount(); ++i) {
            bool isLastChild = i == node->childCount() - 1;
            printNode(os, node->getChildren()[i].get(), childPrefix, isLastChild);
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const Tree<T>& tree) {
        if (tree.isEmpty()) {
            os << "(empty tree)";
            return os;
        }
        tree.printNode(os, tree.m_root.get(), "", true);
        return os;
    }


private:

    void traversalPreOrderRec(TreeNode<T>* node, std::vector<TreeNode<T>*> &result) {
        result.emplace_back(node);
        for (const auto& n: node->getChildren()) {
            traversalPreOrderRec(n.get(), result);
        }
    }

    std::unique_ptr<TreeNode<T>> m_root;
};
