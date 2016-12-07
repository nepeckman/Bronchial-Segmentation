#include "CollisionDetection.h"
#include "BronchialSegmentation.h"
#include <iostream>

int main(int argc, char** argv)
{
  BronchialSegmentation detector(argc, argv);
  std::vector< Node* > graph = detector.getGraph();
  for(std::vector< Node* >::size_type i = 0; i != graph.size(); i++)
  {
    if(graph[i])
    {
      std::cout << *graph[i]->index << std::endl;
    }
  }
  return 0;
}
