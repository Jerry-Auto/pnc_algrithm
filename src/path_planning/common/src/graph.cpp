#include "graph.h"

Graph::Graph(int vertices) : numVertices(vertices) {
    double posInf = std::numeric_limits<double>::infinity();
    adjacencyMatrix.resize(vertices, std::vector<double>(vertices, posInf));
    for (size_t i = 0; i < adjacencyMatrix.size(); i++) {
        adjacencyMatrix[i][i] = 0.0;
    }
}

void Graph::addEdge(int vertex1, int vertex2, double weight) {
    if (vertex1 >= 0 && vertex1 < numVertices && vertex2 >= 0 && vertex2 < numVertices) {
        adjacencyMatrix[vertex1][vertex2] = weight;
        adjacencyMatrix[vertex2][vertex1] = weight;  // 对于无向图，需要对称设置
    } else {
        throw std::out_of_range("Vertex index out of range");
    }
}

void Graph::removeEdge(int vertex1, int vertex2) {
    if (vertex1 >= 0 && vertex1 < numVertices && vertex2 >= 0 && vertex2 < numVertices) {
        adjacencyMatrix[vertex1][vertex2] = std::numeric_limits<double>::infinity();
        adjacencyMatrix[vertex2][vertex1] = std::numeric_limits<double>::infinity();
    } else {
        throw std::out_of_range("Vertex index out of range");
    }
}

double Graph::getWeight(int vertex1, int vertex2) const {
    if (vertex1 >= 0 && vertex1 < numVertices && vertex2 >= 0 && vertex2 < numVertices) {
        return adjacencyMatrix[vertex1][vertex2];
    } else {
        throw std::out_of_range("Vertex index out of range");
    }
}

int Graph::size() const {
    return numVertices;
}

void Graph::print() const {

    const int width = 10;  // 设置每个元素的输出宽度

    for (int i = 0; i < numVertices; i++) {

        for (int j = 0; j < numVertices; j++) {

            std::cout << std::setw(width) << adjacencyMatrix[i][j] << " ";

        }

        std::cout << std::endl;

    }

}