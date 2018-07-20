#include <iostream>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/filter.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::octree::OctreePointCloudVoxelCentroid<Point> Octree;
//typedef pcl::octree::OctreePointCloudSearch<Point> Octree;
typedef pcl::visualization::PCLVisualizer Visualizer;

void computeVoxelCloud(const Octree::Ptr& octree, int depth, PointCloud::Ptr& cloud);
void addVoxels(float s, const PointCloud::Ptr& cloud, Visualizer::Ptr& viewer, bool occ);
void castRay(const Eigen::Vector3f& origin, const Eigen::Vector3f& end, const Octree::Ptr& octree, PointCloud::Ptr& cloud);

int main(int argc, char** argv){

  Visualizer::Ptr viewer (new Visualizer ("Octree Viewer"));

  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr occ_voxel_cloud (new PointCloud);
  PointCloud::Ptr unn_voxel_cloud (new PointCloud);

  int depth;
  float point_size_=1.0;

  std::cerr << "Loading file " << argv[1] << std::endl;
  pcl::io::loadPCDFile<Point> (argv[1], *cloud);
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

  Octree::Ptr octree(new Octree(0.05));
  octree->setInputCloud(cloud);
  octree->defineBoundingBox();
  octree->addPointsFromInputCloud();

  depth = static_cast<int> (octree->getTreeDepth());
  if (depth == 0)
    depth = 1;
  float s = std::sqrt (octree->getVoxelSquaredSideLen (depth)) / 2.0;

  computeVoxelCloud(octree,depth,occ_voxel_cloud);
  addVoxels(s,occ_voxel_cloud,viewer,true);

  Eigen::Vector3f origin(0,0,0);
  Eigen::Vector3f end(1,0,0);
  castRay(origin,end,octree,unn_voxel_cloud);
  addVoxels(s,unn_voxel_cloud,viewer,false);

  pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(cloud);
  viewer->addPointCloud<Point> (cloud, rgb, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "cloud");

  viewer->addCoordinateSystem (0.5);

  while (!viewer->wasStopped()){
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return 0;
}

void computeVoxelCloud(const Octree::Ptr& octree, int depth, PointCloud::Ptr& cloud){
  cloud->points.clear();

  Octree::Iterator tree_it;
  Octree::Iterator tree_it_end = octree->end();

  Point pt_voxel_center;
  std::cout << "===== Extracting data at depth " << depth << "... " << std::flush;
  double start = pcl::getTime ();

  for (tree_it = octree->begin(depth); tree_it!=tree_it_end; ++tree_it){
    // If the iterator is not at the right depth, continue
    if (tree_it.getCurrentOctreeDepth () != (unsigned int) depth)
      continue;

    // Compute the point at the center of the voxel which represents the current OctreeNode
    Eigen::Vector3f voxel_min, voxel_max;
    octree->getVoxelBounds (tree_it, voxel_min, voxel_max);

    pt_voxel_center.x = (voxel_min.x () + voxel_max.x ()) / 2.0f;
    pt_voxel_center.y = (voxel_min.y () + voxel_max.y ()) / 2.0f;
    pt_voxel_center.z = (voxel_min.z () + voxel_max.z ()) / 2.0f;
    cloud->points.push_back (pt_voxel_center);
  }
  double end = pcl::getTime ();
  printf("%lu pts, %.4gs. %.4gs./pt. =====\n", cloud->points.size (), end - start,
         (end - start) / static_cast<double> (cloud->points.size ()));
}

void castRay(const Eigen::Vector3f &origin,
             const Eigen::Vector3f &end,
             const Octree::Ptr &octree,
             PointCloud::Ptr &cloud){
  cloud->points.clear();

  Octree::AlignedPointTVector voxels;
  voxels.clear();
//  octree->getIntersectedVoxelCenters(origin, end, voxels);
  octree->getApproxIntersectedVoxelCentersBySegment(origin, end, voxels, 0.5);
  Point pt;
  std::vector<int> indices;
  std::cerr << "Ray intersected " << voxels.size() << " voxels" << std::endl;
  for(int i =0;i<voxels.size();++i){
    std::cerr << i << ") " << voxels[i].x << " " << voxels[i].y << " " << voxels[i].z << std::endl;
    pt.x = voxels[i].x;
    pt.y = voxels[i].y;
    pt.z = voxels[i].z;
    pt.r = 0;
    pt.g = 0;
    pt.b = 1.0;
    cloud->points.push_back(pt);
//    indices.clear();
//    octree->voxelSearch(pt,indices);
  }

}

void addVoxels(float s, const PointCloud::Ptr &cloud, Visualizer::Ptr &viewer, bool occ){
  char cube_id [50];
  for (size_t i = 0; i < cloud->points.size (); i++){
    double x = cloud->points[i].x;
    double y = cloud->points[i].y;
    double z = cloud->points[i].z;
    if(occ){
      sprintf(cube_id,"occ_cube_%lu",i);
      viewer->addCube(x - s, x + s, y - s, y + s, z - s, z + s,1.0,0.0,0.0,cube_id);
    } else {
      sprintf(cube_id,"unn_cube_%lu",i);
      viewer->addCube(x - s, x + s, y - s, y + s, z - s, z + s,0.0,0.0,1.0,cube_id);
    }
  }
}
