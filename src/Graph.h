// File: Graph.h
#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <queue>
#include <limits>
#include <iostream>
#include <string>
#include <algorithm>

const int INF = 1e9;
const int N = 11; // Number of airports

class AirlineGraph {
private:
    // Adjacency list: vector of vectors of pairs (neighbor, weight)
    std::vector<std::vector<std::pair<int, int>>> adj;
    // List of airport codes
    std::vector<std::string> airportList;

    // Helper to get index from code
    int getIndex(const std::string& code) const {
        for (int i = 0; i < N; ++i) {
            if (airportList[i] == code) {
                return i;
            }
        }
        return -1;
    }

public:
    AirlineGraph() : adj(N), airportList({
        "PNH", "REP", "KOS", "SGN", "HAN", "SIN", "KUL", "MNL", "HKG", "CAN", "CGK"
    }) {} // Constructor initializes adjacency list and airport codes

    // Function to add an undirected edge between two nodes with a given weight
    void addEdge(int u, int v, int weight) {
        if (weight == 0) return; // Skip zero-weight edges
        // Add edge from u to v
        adj[u].push_back({v, weight});
        // Add edge from v to u (undirected graph)
        adj[v].push_back({u, weight});
    }

    // Dijkstra's algorithm to find shortest path from source to all nodes
    // Returns pair of dist and prev vectors
    std::pair<std::vector<int>, std::vector<int>> dijkstra(int src) const {
        std::vector<int> dist(N, INF);
        std::vector<int> prev(N, -1);
        dist[src] = 0;

        // Min-heap priority queue: pair<distance, node>
        std::priority_queue<std::pair<int, int>, 
                            std::vector<std::pair<int, int>>, 
                            std::greater<std::pair<int, int>>> pq;
        pq.push({0, src});

        while (!pq.empty()) {
            // Extract node with minimum distance
            auto [d, u] = pq.top();
            pq.pop();

            // Skip if a shorter path to u was already found
            if (d > dist[u]) continue;

            // Relax all adjacent edges
            for (auto [v, w] : adj[u]) {
                if (dist[v] > dist[u] + w) {
                    // Update distance and previous node
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                    // Push updated distance to priority queue
                    pq.push({dist[v], v});
                }
            }
        }
        return {dist, prev};
    }

    // Function to print the shortest path from source to destination using prev array
    void printPath(const std::vector<int>& prev, int dest, int src) const {
        std::vector<int> path;
        // Backtrack from destination to source using prev
        for (int at = dest; at != -1; at = prev[at]) {
            path.push_back(at);
        }
        // Safety check: Ensure we reached the actual source
        if (path.empty() || path.back() != src) {
            std::cout << "No path exists." << std::endl;
            return;
        }
        // Print path with arrows (MANUAL REVERSE LOOP - no std::reverse, no duplication)
        for (int i = (int)path.size() - 1; i >= 0; --i) {
            std::cout << airportList[path[i]];
            if (i > 0) {
                std::cout << " -> ";
            }
        }
        std::cout << std::endl;
    }

    // Print all available airports
    void printAirports() const {
        std::cout << "Available Airports: ";
        for (size_t i = 0; i < airportList.size(); ++i) {
            std::cout << airportList[i];
            if (i < airportList.size() - 1) std::cout << " ";
        }
        std::cout << std::endl;
    }

    // Print all direct routes (undirected, print each once)
    void printRoutes() const {
        std::cout << "\nDirect Routes:" << std::endl;
        for (int u = 0; u < N; ++u) {
            for (auto [v, w] : adj[u]) {
                if (u < v) { // Avoid duplicates
                    std::cout << airportList[u] << " <-> " 
                              << airportList[v] << " = " << w << std::endl;
                }
            }
        }
    }

    // Find and print shortest path from source to destination
    void findShortestPath(const std::string& source, const std::string& dest) const {
        int s = getIndex(source);
        int t = getIndex(dest);

        // Validate input
        if (s == -1 || t == -1 || s == t) {
            std::cout << "Invalid input or source equals destination." << std::endl;
            return;
        }

        // Run Dijkstra's algorithm
        auto [dist, prev] = dijkstra(s);

        // Output results
        if (dist[t] == INF) {
            std::cout << "No path from " << source << " to " << dest << "." << std::endl;
        } else {
            std::cout << "\nShortest distance from " << source << " to " << dest 
                      << ": " << dist[t] << std::endl;
            std::cout << "Full route: ";
            printPath(prev, t, s);
        }
    }
};

#endif // GRAPH_H