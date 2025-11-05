#pragma once
#include <cassert>

#include "map_loader.h"
#include "../datastructure/graph.h"

inline double roundToGrid(const double value, const double gridSize,const bool up = true) {
    if (up) {
        return std::ceil(value / gridSize) * gridSize;
    }
    return std::floor(value / gridSize) * gridSize;
}

inline void connectAllGridNodes(Graph &gridGraph, const std::vector<std::vector<Node> > &lines) {
    Log::d("[ MapView ] Connecting all Grid nodes by edges");
    std::vector < Node > prevLine;
    for (auto l: lines) {
        for (int j = 0; j < l.size(); ++j) {
            // horizontal
            if (j + 1 < l.size()) {
                gridGraph.addEdge(l[j].id, l[j + 1].id);
            }
            // vertical
            if (!prevLine.empty()) {
                gridGraph.addEdge(prevLine[j].id, l[j].id);
            }
        }
        prevLine = l;
    }
}

inline void createGridGraph(
    const double gridSize,
    const double &offsetX,
    const double &offsetY,
    const double h,
    const double w,
    Graph &gridGraph,
    std::vector<std::vector<Node>> &lines
    ) {
    Log::d("[ MapView ] Create Grid Graph");

    const int rows = static_cast<int>(h / gridSize + 1);
    const int cols = static_cast<int>(w / gridSize + 1);

    lines = {};
    for (int i = 0; i <= rows; ++i) {
        const double y = offsetY + gridSize * i;

        std::vector<Node> currentLine = {};
        for (int j = 0; j <= cols; ++j) {
            const double x = offsetX + gridSize * j;
            Node n(x, y);
            gridGraph.addNode(n);
            currentLine.emplace_back(n);
        }
        lines.emplace_back(currentLine);
    }
}

inline std::vector<Node> removeIntersectingEdges(
    Graph &gridGraph,
    const std::vector<std::vector<Node>>& girdLines,
    const std::vector<VisLib::Segment>& segments,
    const double offsetX,
    const double offsetY,
    const double gridSize
    ) {
    Log::d("[ MapView ] Remove intersecting segments edges");
    std::vector<Node> visitedNodes;
    for (auto& segment: segments) {
        //drawPath({segment.m_points[0].m_x, segment.m_points[0].m_y},{segment.m_points[1].m_x, segment.m_points[1].m_y},Qt::magenta);

        double gridX1 = segment.m_points[0].m_x - offsetX;
        double gridY1 = segment.m_points[0].m_y - offsetY;
        double gridX2 = segment.m_points[1].m_x - offsetX;
        double gridY2 = segment.m_points[1].m_y - offsetY;
        //std::cout << gridX1 << ", " << gridY1 <<  "), (" << gridX2 << ", " << gridY2 << ")" << std::endl;

        gridX1 = roundToGrid(gridX1, gridSize, gridX1 > gridX2);
        gridX2 = roundToGrid(gridX2, gridSize, gridX2 > gridX1);
        gridY1 = roundToGrid(gridY1, gridSize, gridY1 > gridY2);
        gridY2 = roundToGrid(gridY2, gridSize, gridY2 > gridY1);

        //std::cout << "("<< gridX1 << ", " << gridY1 <<  "), (" << gridX2 << ", " << gridY2 << ")" << std::endl;

        int lineX1 = static_cast<int>(gridX1 / gridSize);
        int lineX2 = static_cast<int>(gridX2 / gridSize);
        int lineY1 = static_cast<int>(gridY1 / gridSize);
        int lineY2 = static_cast<int>(gridY2 / gridSize);

        auto [minX, maxX] = std::minmax(lineX1, lineX2);
        auto [minY, maxY] = std::minmax(lineY1, lineY2);

        //std::cout << "("<<lineX1 << ", " << lineY1 <<  "), (" << lineX2 << ", " << lineY2 << ")" << std::endl;

        for (int j = minY; j <= maxY; ++j) {
            for (int i = minX; i <= maxX; ++i) {
                assert(j < girdLines.size());
                assert(i < girdLines[0].size());
                auto n = girdLines[j][i];
                auto neighbours = gridGraph.neighbours(n.id);
                for (const auto neighbour: neighbours) {
                    auto currentNeighbour = gridGraph.getNode(neighbour);
                    if (VisLib::intersect(segment, {n.x, n.y}, { currentNeighbour.x, currentNeighbour.y })) {
                        gridGraph.removeEdge(n.id, neighbour);
                    }
                }
                visitedNodes.emplace_back(n);
            }
        }
    }
    return visitedNodes;
}

inline int getNearestNode(const Graph& graph, const double x, const double y) {
    double minDist = std::numeric_limits<double>::max();
    int nearestNodeId = 0;
    for (auto [_, n]: graph.allNodes()) {
       double dist = util::calculateDistance(n.x, n.y, x, y);
        if (dist < minDist) {
            minDist = dist;
            nearestNodeId = n.id;
        }
    }
    return nearestNodeId;
}