#include <gtest/gtest.h>
#include "../datastructure/tree.h"

class TreeTest : public ::testing::Test {
protected:
    Tree<int> tree;

    void SetUp() override {
        auto* root = tree.createRoot(0);
        TreeNode node(1, root);
        root->addChild(1);
        tree.getAllNodes();
    }

    void TearDown() override {
    }
};

TEST_F(TreeTest, SimpleTest) {
    // EXPECT_EQ(graph.nodeCount(), 3);
    // graph.addNode(1.0, 1.0);
    // EXPECT_EQ(graph.nodeCount(), 4);
}