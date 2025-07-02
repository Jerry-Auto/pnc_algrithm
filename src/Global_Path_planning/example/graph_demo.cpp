#include"graph.h"
#include <iostream>
int main()
{
    Graph test_graph(4);
    test_graph.addEdge(0, 1, 5.0);
    test_graph.addEdge(1, 2, 3.0);
    test_graph.addEdge(2, 3, 7.0);
    std::cout << "Adjacency Matrix:" << std::endl;
    test_graph.print();
    return 0;
}