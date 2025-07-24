#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <stdexcept>
#include <limits>  // 用于 std::numeric_limits
#include <iomanip>  // 用于 std::setw

class Graph {
private:
    int numVertices;
    std::vector<std::vector<double>> adjacencyMatrix;

public:
    Graph(int vertices);
    void addEdge(int vertex1, int vertex2, double weight = 1.0);
    void removeEdge(int vertex1, int vertex2);
    double getWeight(int vertex1, int vertex2) const;
    int size() const;
    void print() const;
};

#endif // GRAPH_H