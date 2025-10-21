#include <gtest/gtest.h>
#include "../datastructure/tree.h"

class TreeTest : public ::testing::Test {
protected:
    Tree<int> tree;

    void SetUp() override {
    }

    void TearDown() override {
        tree.clear();
    }
};

TEST_F(TreeTest, EmptyTreeTest) {
    EXPECT_TRUE(tree.isEmpty());
    EXPECT_EQ(tree.getRoot(), nullptr);
}

TEST_F(TreeTest, CreateRootTest) {
    auto* root = tree.createRoot(10);

    EXPECT_FALSE(tree.isEmpty());
    EXPECT_NE(root, nullptr);
    EXPECT_EQ(root->getValue(), 10);
    EXPECT_EQ(tree.getRoot(), root);
}

TEST_F(TreeTest, RootHasNoParent) {
    auto* root = tree.createRoot(5);

    EXPECT_EQ(root->parent, nullptr);
}

TEST_F(TreeTest, AddSingleChild) {
    auto* root = tree.createRoot(1);
    auto* child = root->addChild(2);

    EXPECT_NE(child, nullptr);
    EXPECT_EQ(child->getValue(), 2);
    EXPECT_EQ(child->parent, root);
    EXPECT_EQ(root->childCount(), 1);
}

TEST_F(TreeTest, AddMultipleChildren) {
    auto* root = tree.createRoot(1);
    auto* child1 = root->addChild(2);
    auto* child2 = root->addChild(3);
    auto* child3 = root->addChild(4);

    EXPECT_EQ(root->childCount(), 3);
    EXPECT_EQ(child1->parent, root);
    EXPECT_EQ(child2->parent, root);
    EXPECT_EQ(child3->parent, root);
}

TEST_F(TreeTest, AddGrandchildren) {
    auto* root = tree.createRoot(1);
    auto* child = root->addChild(2);
    auto* grandchild = child->addChild(3);

    EXPECT_EQ(grandchild->parent, child);
    EXPECT_EQ(child->parent, root);
    EXPECT_EQ(child->childCount(), 1);
}

TEST_F(TreeTest, RootIsLeafWhenNoChildren) {
    auto* root = tree.createRoot(1);

    EXPECT_TRUE(root->isLeaf());
}

TEST_F(TreeTest, NodeIsNotLeafWhenHasChildren) {
    auto* root = tree.createRoot(1);
    root->addChild(2);

    EXPECT_FALSE(root->isLeaf());
}

TEST_F(TreeTest, ChildIsLeaf) {
    auto* root = tree.createRoot(1);
    auto* child = root->addChild(2);

    EXPECT_TRUE(child->isLeaf());
}

TEST_F(TreeTest, GetAllNodesEmptyTree) {
    auto nodes = tree.getAllNodes();

    EXPECT_TRUE(nodes.empty());
}

TEST_F(TreeTest, GetAllNodesSingleNode) {
    tree.createRoot(10);
    auto nodes = tree.getAllNodes();

    EXPECT_EQ(nodes.size(), 1);
    EXPECT_EQ(nodes[0]->getValue(), 10);
}

TEST_F(TreeTest, GetAllNodesMultipleNodes) {
    auto* root = tree.createRoot(1);
    root->addChild(2);
    root->addChild(3);

    auto nodes = tree.getAllNodes();

    EXPECT_EQ(nodes.size(), 3);
}

TEST_F(TreeTest, GetAllNodesComplexTree) {
    auto* root = tree.createRoot(1);
    auto* child1 = root->addChild(2);
    auto* child2 = root->addChild(3);
    child1->addChild(4);
    child1->addChild(5);
    child2->addChild(6);

    auto nodes = tree.getAllNodes();

    EXPECT_EQ(nodes.size(), 6);
}

TEST_F(TreeTest, GetDescendantsOfLeaf) {
    auto* root = tree.createRoot(1);
    auto* child = root->addChild(2);

    auto descendants = child->getAllDescendants();

    EXPECT_TRUE(descendants.empty());
}

TEST_F(TreeTest, GetDescendantsOfRoot) {
    auto* root = tree.createRoot(1);
    root->addChild(2);
    root->addChild(3);

    auto descendants = root->getAllDescendants();

    EXPECT_EQ(descendants.size(), 2);
}

TEST_F(TreeTest, GetDescendantsRecursive) {
    auto* root = tree.createRoot(1);
    auto* child1 = root->addChild(2);
    root->addChild(3);
    child1->addChild(4);
    child1->addChild(5);

    auto descendants = root->getAllDescendants();

    EXPECT_EQ(descendants.size(), 4);  // 2, 3, 4, 5
}

TEST_F(TreeTest, RemoveChild) {
    auto* root = tree.createRoot(1);
    auto* child1 = root->addChild(2);
    auto* child2 = root->addChild(3);

    root->removeChild(child1);

    EXPECT_EQ(root->childCount(), 1);
    EXPECT_EQ(root->getChildren()[0]->getValue(), 3);
}

TEST_F(TreeTest, RemoveSubtree) {
    auto* root = tree.createRoot(1);
    auto* child = root->addChild(2);
    child->addChild(3);
    child->addChild(4);

    tree.removeSubtree(child);

    EXPECT_EQ(root->childCount(), 0);
    EXPECT_TRUE(root->isLeaf());
}

TEST_F(TreeTest, RemoveRootClearsTree) {
    auto* root = tree.createRoot(1);
    root->addChild(2);

    tree.removeSubtree(root);

    EXPECT_TRUE(tree.isEmpty());
}

TEST_F(TreeTest, ClearTree) {
    auto* root = tree.createRoot(1);
    root->addChild(2);
    root->addChild(3);

    tree.clear();

    EXPECT_TRUE(tree.isEmpty());
}

TEST_F(TreeTest, ComplexTreeStructure) {
    /*
     * Struktur:
     *       1
     *      / \
     *     2   3
     *    / \   \
     *   4   5   6
     */
    auto* root = tree.createRoot(1);
    auto* child1 = root->addChild(2);
    auto* child2 = root->addChild(3);
    child1->addChild(4);
    child1->addChild(5);
    child2->addChild(6);

    EXPECT_EQ(root->childCount(), 2);
    EXPECT_EQ(child1->childCount(), 2);
    EXPECT_EQ(child2->childCount(), 1);

    auto allNodes = tree.getAllNodes();
    EXPECT_EQ(allNodes.size(), 6);
}

TEST_F(TreeTest, DeepTree) {
    auto* node1 = tree.createRoot(1);
    auto* node2 = node1->addChild(2);
    auto* node3 = node2->addChild(3);
    auto* node4 = node3->addChild(4);
    auto* node5 = node4->addChild(5);

    EXPECT_EQ(tree.getAllNodes().size(), 5);
    EXPECT_TRUE(node5->isLeaf());
    EXPECT_EQ(node5->parent, node4);
}

TEST_F(TreeTest, AddChildToLeaf) {
    auto* root = tree.createRoot(1);
    auto* child = root->addChild(2);

    EXPECT_TRUE(child->isLeaf());

    child->addChild(3);

    EXPECT_FALSE(child->isLeaf());
}

TEST_F(TreeTest, MultipleRootCreations) {
    tree.createRoot(1);
    tree.createRoot(2);

    EXPECT_EQ(tree.getRoot()->getValue(), 2);
    EXPECT_EQ(tree.getAllNodes().size(), 1);
}

TEST_F(TreeTest, RemoveNonExistentChild) {
    auto* root = tree.createRoot(1);
    auto* child = root->addChild(2);

    Tree<int> otherTree;
    auto* otherNode = otherTree.createRoot(99);

    root->removeChild(otherNode);

    EXPECT_EQ(root->childCount(), 1);
}


class StringTreeTest : public ::testing::Test {
protected:
    Tree<std::string> tree;
};

TEST_F(StringTreeTest, StringValues) {
    auto* root = tree.createRoot("root");
    auto* child1 = root->addChild("child1");
    auto* child2 = root->addChild("child2");

    EXPECT_EQ(root->getValue(), "root");
    EXPECT_EQ(child1->getValue(), "child1");
    EXPECT_EQ(child2->getValue(), "child2");
}

TEST_F(StringTreeTest, EmptyStrings) {
    auto* root = tree.createRoot("");
    auto* child = root->addChild("");

    EXPECT_EQ(root->getValue(), "");
    EXPECT_EQ(child->getValue(), "");
}