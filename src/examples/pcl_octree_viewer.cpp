#include <iostream>

#include "octree_viewer.h"

int main(int argc, char** argv){

  if (argc != 3){
    std::cerr << "ERROR: Syntax is pcl_octree_viewer <pcd file> <resolution>" << std::endl;
    std::cerr << "EXAMPLE: ./octreeVisu bun0.pcd 0.001" << std::endl;
    return -1;
  }

  std::string cloud_path(argv[1]);
  OctreeViewer v(cloud_path, atof(argv[2]));

  return 0;
}
