#include <iostream>
#include "Graph.h"

int main(){
    std::cout << "Hello graph" << std::endl;

    DirectedGraph<std::string, std::string> directedG;

    directedG.import("inp.txt");

    directedG.print();
    return 0;
}