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
    std::vector<std::vector<std::pair<int, int> > > adj;
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

    // DFS helper to find all paths between source and destination
    void dfsAllPaths(int current, int dest, std::vector<bool>& visited,
                     std::vector<int>& path, std::vector<std::vector<int> >& allPaths) const {
        visited[current] = true;
        path.push_back(current);

        if (current == dest) {
            allPaths.push_back(path);
        } else {
            for(const auto& [neighbor, weight] : adj[current]) {
                if (!visited[neighbor]) {
                    dfsAllPaths(neighbor, dest, visited, path, allPaths);
                }
            }
        }

        path.pop_back();
        visited[current] = false;
    }

    // Calculate total distance for a path
    int calculatePathDistance(const std::vector<int>& path) const {
        int totalDist = 0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            int u = path[i];
            int v = path[i + 1];
            // Find the edge weight between u and v
            for (const auto& [neighbor, weight] : adj[u]) {
                if (neighbor == v) {
                    totalDist += weight;
                    break;
                }
            }
        }
        return totalDist;
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

    // NEW FUNCTION 1: Find path with minimum stops between two airports
    void findMinimumStops(const std::string& source, const std::string& dest) const {
        int s = getIndex(source);
        int t = getIndex(dest);

        // Validate input
        if (s == -1 || t == -1) {
            std::cout << "Invalid airport code(s)." << std::endl;
            return;
        }

        if (s == t) {
            std::cout << "Source and destination are the same." << std::endl;
            return;
        }

        // Find all paths using DFS
        std::vector<bool> visited(N, false);
        std::vector<int> currentPath;
        std::vector<std::vector<int>> allPaths;

        dfsAllPaths(s, t, visited, currentPath, allPaths);

        if (allPaths.empty()) {
            std::cout << "No path from " << source << " to " << dest << "." << std::endl;
            return;
        }

        // Find the path with minimum stops
        int minStops = INF;
        int minPathIndex = 0;

        for (size_t i = 0; i < allPaths.size(); ++i) {
            int stops = allPaths[i].size() - 1;
            if (stops < minStops) {
                minStops = stops;
                minPathIndex = i;
            }
        }

        const std::vector<int>& shortestPath = allPaths[minPathIndex];
        int totalDistance = calculatePathDistance(shortestPath);

        // Output results
        std::cout << "\nPath with Minimum Stops from " << source << " to " << dest << ":" << std::endl;
        std::cout << "Minimum stops: " << minStops << std::endl;
        std::cout << "Total distance: " << totalDistance << std::endl;
        std::cout << "Route: ";

        for (size_t i = 0; i < shortestPath.size(); ++i) {
            std::cout << airportList[shortestPath[i]];
            if (i < shortestPath.size() - 1) {
                std::cout << " -> ";
            }
        }
        std::cout << std::endl;
    }

    // NEW FUNCTION 2: Find all possible paths between two airports
    void findAllPossiblePaths(const std::string& source, const std::string& dest) const {
        int s = getIndex(source);
        int t = getIndex(dest);

        // Validate input
        if (s == -1 || t == -1) {
            std::cout << "Invalid airport code(s)." << std::endl;
            return;
        }

        if (s == t) {
            std::cout << "Source and destination are the same." << std::endl;
            return;
        }

        // Find all paths using DFS
        std::vector<bool> visited(N, false);
        std::vector<int> currentPath;
        std::vector<std::vector<int>> allPaths;

        dfsAllPaths(s, t, visited, currentPath, allPaths);

        if (allPaths.empty()) {
            std::cout << "No path from " << source << " to " << dest << "." << std::endl;
            return;
        }

        // Output all paths
        std::cout << "\nAll Possible Paths from " << source << " to " << dest << ":" << std::endl;
        std::cout << "Total paths found: " << allPaths.size() << "\n" << std::endl;

        for (size_t idx = 0; idx < allPaths.size(); ++idx) {
            const std::vector<int>& path = allPaths[idx];
            int totalDistance = calculatePathDistance(path);
            int stops = path.size() - 1;

            std::cout << "Path " << (idx + 1) << ": ";

            for (size_t i = 0; i < path.size(); ++i) {
                std::cout << airportList[path[i]];
                if (i < path.size() - 1) {
                    std::cout << " -> ";
                }
            }

            std::cout << " | Distance: " << totalDistance 
                      << " | Stops: " << stops << std::endl;
        }
    }
};

#endif // GRAPH_H