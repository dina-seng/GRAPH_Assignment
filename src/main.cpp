// File: main.cpp
#include <iostream>
#include <string>
#include "Graph.h"

int main() {
    // Create graph instance and construct with hardcoded sample data (undirected edges with averaged/symmetric weights where applicable)
    AirlineGraph graph;
    // PNH (0) edges
    graph.addEdge(0, 1, 205);  // PNH ↔ REP = 205 (averaged)
    graph.addEdge(0, 2, 180);  // PNH ↔ KOS = 180
    graph.addEdge(0, 3, 210);  // PNH ↔ SGN = 210
    graph.addEdge(0, 4, 1050); // PNH ↔ HAN = 1050
    graph.addEdge(0, 5, 1150); // PNH ↔ SIN = 1150
    graph.addEdge(0, 6, 1030); // PNH ↔ KUL = 1030
    // REP (1) additional
    graph.addEdge(1, 3, 370);  // REP ↔ SGN = 370 (averaged)
    graph.addEdge(1, 5, 1350); // REP ↔ SIN = 1350
    // KOS (2) additional
    graph.addEdge(2, 3, 320);  // KOS ↔ SGN = 320
    // SGN (3) additional
    graph.addEdge(3, 4, 1150); // SGN ↔ HAN = 1150
    graph.addEdge(3, 5, 1080); // SGN ↔ SIN = 1080
    graph.addEdge(3, 9, 1800); // SGN ↔ CAN = 1800
    // HAN (4) additional
    graph.addEdge(4, 8, 250);  // HAN ↔ HKG = 250
    // SIN (5) additional
    graph.addEdge(5, 6, 300);  // SIN ↔ KUL = 300
    graph.addEdge(5, 7, 2490); // SIN ↔ MNL = 2490
    graph.addEdge(5, 8, 250);  // SIN ↔ HKG = 250
    // KUL (6) additional
    graph.addEdge(6, 7, 2490); // KUL ↔ MNL = 2490
    graph.addEdge(6, 9, 1140); // KUL ↔ CAN = 1140 (one-sided, symmetrized)
    // MNL (7) additional
    graph.addEdge(7, 8, 1140); // MNL ↔ HKG = 1140
    // HKG (8) additional
    graph.addEdge(8, 9, 130);  // HKG ↔ CAN = 130
    graph.addEdge(8, 10, 3270); // HKG ↔ CGK = 3270
    // CAN (9) already covered

    std::string choiceStr;
    int choice;

    std::cout << "=== Airline Route Planner ===" << std::endl;

    do {
        // Display menu
        std::cout << "\nMenu Options:" << std::endl;
        std::cout << "1. Display all available airports" << std::endl;
        std::cout << "2. Display all direct routes" << std::endl;
        std::cout << "3. Find shortest path between two airports" << std::endl;
        std::cout << "4. Exit" << std::endl;
        std::cout << "Enter your choice (1-4): ";
        std::cin >> choiceStr;
        // Simple conversion to int (assume valid input for simplicity)
        choice = std::stoi(choiceStr);

        switch (choice) {
            case 1:
                graph.printAirports();
                break;
            case 2:
                graph.printRoutes();
                break;
            case 3: {
                std::string source, dest;
                std::cout << "Enter source airport code (e.g., PNH): ";
                std::cin >> source;
                std::cout << "Enter destination airport code (e.g., SIN): ";
                std::cin >> dest;
                graph.findShortestPath(source, dest);
                break;
            }
            case 4:
                std::cout << "Goodbye!" << std::endl;
                break;
            default:
                std::cout << "Invalid choice. Please try again." << std::endl;
        }
    } while (choice != 4);

    return 0;
}