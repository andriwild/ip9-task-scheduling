#pragma once
#include <cmath>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>
#include "../util/log.h"

using Edge = std::pair<int, int>;

static int nodeId = 0;


struct Node {
    int id;
    double x;
    double y;

    Node(const double x, const double y):
    id(nodeId++), x(x), y(y) {}

    double distanceTo(const Node& other) const {
        const double dx = x - other.x;
        const double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};


class Graph {
    std::unordered_map<int, Node> m_nodes;
    std::unordered_map<int, std::vector<int>> m_adjList;

public:
    int addNode(const double x, const double y) {
        auto newNode = Node(x,y);
        m_nodes.emplace(newNode.id, newNode);
        return newNode.id;
    }

    int addNode(const Node &n) {
        m_nodes.emplace(n.id, n);
        return n.id;
    }

    void addEdge(const int fromId, const int toId) {
        m_adjList[fromId].push_back(toId);
        m_adjList[toId].push_back(fromId);
    }

    size_t nodeCount() const {
        return m_nodes.size();
    }

    void clear() {
        m_nodes.clear();
        m_adjList.clear();
        nodeId = 0;
    }

    std::unordered_map<int, Node> allNodes() const {
        return m_nodes;
    }

    std::unordered_map<int, std::vector<int>> allEdges() const {
        return m_adjList;
    }

    Node getNode(const int id) const {
        if (m_nodes.contains(id)) {
           return m_nodes.at(id);
        }
        std::cout << "searched for id: " << id << std::endl;
        std::__throw_range_error("node not found");
    }

    struct DijkstraResult {
        // distances to all other nodes
        // id, distance
        std::unordered_map<int, double> distances;
        // id, id
        std::unordered_map<int, int> previous;

        std::vector<int> shortestPath(const int targetId) {
            std::vector<int> path;
            bool hasEntry = distances.find(targetId) != distances.end();
            bool hasDistance = distances.at(targetId) != std::numeric_limits<double>::infinity();
            // no path found for this node
            if (!hasEntry || !hasDistance) {
                return path;
            }

            // go backwards until start node is found
            int current = targetId;
            bool present = previous.find(current) != previous.end();
            while (present) {
                path.push_back(current);
                current = previous.at(current);
                present = previous.find(current) != previous.end();

            }
            path.push_back(current);
            std::reverse(path.begin(), path.end());
            return path;
        }

        void print() {
            // for (auto d: distances) {
            //     std::cout << d.first << " -> " << d.second << std::endl;
            // }

            for (auto d: previous) {
                std::cout << d.first << " -> " << d.second << std::endl;
            }
        }
    };

    DijkstraResult dijkstra(const int nodeId) {
        assert(m_nodes.contains(nodeId));
        DijkstraResult result;

        for (auto [id, node]: m_nodes) {
            result.distances[id] = std::numeric_limits<double>::infinity();
        }

        // starting point
        result.distances[nodeId] = 0.0;

        using Elem = std::pair<double, int>;
        std::priority_queue<Elem, std::vector<Elem>, std::greater<>> pq;
        Elem e(0.0, nodeId);
        pq.emplace(e);

        //result.print();
        while (!pq.empty()) {
            auto [currentDistance, id] = pq.top();
            pq.pop();

            // check, if node has some neighbours
            auto it = m_adjList.find(id);
            if (it != m_adjList.end()) {

                // iterate through all neighbours
                for (auto neighbourId: m_adjList.at(id)) {
                    Node to   = m_nodes.at(neighbourId);
                    Node from = m_nodes.at(id);
                    double distance = from.distanceTo(to) + currentDistance;

                    // do we already know a shorter path to the node?
                    if (distance < result.distances[neighbourId]) {
                        result.distances[neighbourId] = distance;
                        result.previous[neighbourId] = id;
                        Elem ne(distance, neighbourId);
                        pq.emplace(ne);
                    }
                }
            }
        }
        return result;
    }


    friend std::ostream& operator<<(std::ostream& os, const Graph& graph) {
        os << "Graph[" << graph.m_nodes.size() << " nodes]\n";

        for (const auto& [id, node] : graph.m_nodes) {
            os << "  [" << id << "] (" << node.x << "," << node.y << "): ";

            auto it  = graph.m_adjList.find(id);
            if (it == graph.m_adjList.end()) {
                os << "isolated";
            } else {
                const auto& neighbors = graph.m_adjList.at(id);
                bool first = true;
                for (int neighborId : neighbors) {
                    if (!first) os << ", ";
                    os << neighborId;
                    first = false;
                }
            }
            os << "\n";
        }
        return os;
    }
};