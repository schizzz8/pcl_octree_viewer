add_executable (pcl_octree_viewer
  pcl_octree_viewer.cpp
  octree_viewer.cpp octree_viewer.h
)

target_link_libraries (pcl_octree_viewer ${PCL_LIBRARIES})

