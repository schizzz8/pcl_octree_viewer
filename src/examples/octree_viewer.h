#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/filter.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

class OctreeViewer
{
public:
  OctreeViewer (std::string &filename, double resolution);

private:

  //visualizer
  pcl::visualization::PCLVisualizer viz;
  //original cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  //displayed_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud;
  // cloud which contains the voxel center
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
  //octree
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;
  //level
  int displayedDepth;
  //bool to decide what should be display
  bool wireframe;
  bool show_cubes_, show_centroids_, show_original_points_;
  float point_size_;
  //========================================================

  /* \brief Callback to interact with the keyboard
   *
   */
  void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void *);


  /* \brief Graphic loop for the viewer
   *
   */
  void run();


  /* \brief Helper function that read a pointcloud file (returns false if pbl)
   *  Also initialize the octree
   *
   */
  bool loadCloud(std::string &filename);

  /* \brief Helper function that draw info for the user on the viewer
   *
   */
  void showLegend ();

  /* \brief Visual update. Create visualizations and add them to the viewer
   *
   */
  void update();

  /* \brief remove dynamic objects from the viewer
   *
   */
  void clearView();


  /* \brief display octree cubes via vtk-functions
   *
   */
  void showCubes(double voxelSideLen);


  /* \brief Extracts all the points of depth = level from the octree
   *
   */
  void extractPointsAtLevel(int depth);

  /* \brief Helper function to increase the octree display level by one
   *
   */
  bool IncrementLevel();

  /* \brief Helper function to decrease the octree display level by one
   *
   */
  bool DecrementLevel();

};
