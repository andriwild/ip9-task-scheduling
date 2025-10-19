#include <gtest/gtest.h>
#include "../datastructure/graph.h"

class GraphTest : public ::testing::Test {
        protected:
            Graph graph;

            void SetUp() override {
                graph.addNode(0.0, 0.0);
                graph.addNode(3.0, 4.0);
                graph.addNode(6.0, 0.0);
            }

            void TearDown() override {
                graph.clear();
            }
        };

TEST_F(GraphTest, AddNodeIncreasesCount) {
    EXPECT_EQ(graph.nodeCount(), 3);
    graph.addNode(1.0, 1.0);
    EXPECT_EQ(graph.nodeCount(), 4);
}

TEST_F(GraphTest, ClearGraphDecreasesCount) {
    EXPECT_EQ(graph.nodeCount(), 3);
    graph.clear();
    EXPECT_EQ(graph.nodeCount(), 0);
}

TEST_F(GraphTest, DijkstraWithOneNode) {
    graph.clear();
    graph.addNode({.0,.0});
    auto [distances, previous] = graph.dijkstra(0);
    EXPECT_EQ(distances[0], 0.0);
    EXPECT_EQ(previous.empty(), true);
}

TEST_F(GraphTest, DijkstraSimple) {
    graph.clear();
    graph.addNode({.0,.0});
    graph.addNode({100.0, .0});
    graph.addEdge(0,1);

    auto [distances, previous] = graph.dijkstra(0);
    EXPECT_EQ(distances[0], 0.0);
    EXPECT_EQ(distances[1], 100.0);
    EXPECT_EQ(previous[1], 0);
}
