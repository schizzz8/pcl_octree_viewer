#include <iostream>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

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
//typedef pcl::octree::OctreePointCloudVoxelCentroid<Point> Octree;
typedef pcl::octree::OctreePointCloudSearch<Point> Octree;
typedef pcl::visualization::PCLVisualizer Visualizer;

void computeVoxelCloud(const Octree::Ptr& octree, int depth, PointCloud::Ptr& cloud);
void castRay(const Eigen::Vector3f& origin, const Eigen::Vector3f& end, const Octree::Ptr& octree, PointCloud::Ptr& cloud);
void showCubes(double side,
               const PointCloud::Ptr& occ_cloud,
               const PointCloud::Ptr& fre_cloud,
               const PointCloud::Ptr& unn_cloud,
               Visualizer::Ptr& viewer);

int main(int argc, char** argv){

  Visualizer::Ptr viewer (new Visualizer ("Octree Viewer"));

  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr occ_voxel_cloud (new PointCloud);
  PointCloud::Ptr fre_voxel_cloud (new PointCloud);
  PointCloud::Ptr unn_voxel_cloud (new PointCloud);

  int depth;
  float point_size_=1.0;
  float resolution = 0.05;
  float inv_resolution = 1./resolution;

  std::cerr << "Loading file " << argv[1] << " ";
  pcl::io::loadPCDFile<Point> (argv[1], *cloud);
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "with: " << cloud->points.size() << " points" << std::endl;

  Octree::Ptr octree(new Octree(resolution));
  octree->setInputCloud(cloud);
  octree->defineBoundingBox();
  octree->addPointsFromInputCloud();

  depth = static_cast<int> (octree->getTreeDepth());
  if (depth == 0)
    depth = 1;
  float s = std::sqrt (octree->getVoxelSquaredSideLen (depth)) / 2.0;

//  computeVoxelCloud(octree,depth,occ_voxel_cloud);

  std::cerr << "Octree: " << std::endl;
  std::cerr << ">>Depth: " << depth << std::endl;
  std::cerr << ">>Resolution: " << s*2 << std::endl;
  std::cerr << ">>Voxels: " << occ_voxel_cloud->size() << std::endl;

  Point min_pt,max_pt;
  pcl::getMinMax3D(*cloud,min_pt,max_pt);
  Eigen::Vector3f min(min_pt.x,min_pt.y,min_pt.z);
  Eigen::Vector3f max(max_pt.x,max_pt.y,max_pt.z);

  std::cerr << ">>Min: " << min.transpose() << std::endl;
  std::cerr << ">>Max: " << max.transpose() << std::endl;

  Eigen::Vector3i dim((int)((max.x()-min.x())*inv_resolution),
                      (int)((max.y()-min.y())*inv_resolution),
                      (int)((max.z()-min.z())*inv_resolution));
  std::cerr << ">>Dim: " << dim.transpose() << std::endl;
  Eigen::Vector3f origin,end;
  Octree::AlignedPointTVector voxels;
  for(int r=0; r<dim.z(); ++r)
    for(int c=0; c<dim.y(); ++c){
      origin.x() = min.x();
      origin.y() = min.y()+c*resolution;
      origin.z() = min.z()+r*resolution;
      end.x() = max.x();
      end.y() = min.y()+c*resolution;
      end.z() = min.z()+r*resolution;

      voxels.clear();
      octree->getApproxIntersectedVoxelCentersBySegment(origin, end, voxels, 0.5);

      Point pt;
      std::vector<int> indices;
      bool hit=false;
      for(int i =0;i<voxels.size();++i){
        pt.x = voxels[i].x;
        pt.y = voxels[i].y;
        pt.z = voxels[i].z;
        pt.r = 0;
        pt.g = 0;
        pt.b = 1.0;
        indices.clear();
        bool found=octree->voxelSearch(pt,indices);
        if(!found){
          if(hit){
            unn_voxel_cloud->points.push_back(pt);
            continue;
          }
          fre_voxel_cloud->points.push_back(pt);
          continue;
        }
        if(indices.size()){
          occ_voxel_cloud->points.push_back(pt);
          if(!hit)
            hit=true;
          continue;
        }
      }
    }

  showCubes(s,occ_voxel_cloud,fre_voxel_cloud,unn_voxel_cloud,viewer);

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
}

void castRay(const Eigen::Vector3f &origin,
             const Eigen::Vector3f &end,
             const Octree::Ptr &octree,
             PointCloud::Ptr &cloud){
  cloud->points.clear();

  Octree::AlignedPointTVector voxels;
  voxels.clear();
//    octree->getIntersectedVoxelCenters(origin, end, voxels);
  octree->getApproxIntersectedVoxelCentersBySegment(origin, end, voxels, 0.5);
  Point pt;
  std::vector<int> indices;
  for(int i =0;i<voxels.size();++i){
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

void showCubes(double s,
               const PointCloud::Ptr& occ_cloud,
               const PointCloud::Ptr& fre_cloud,
               const PointCloud::Ptr& unn_cloud,
               Visualizer::Ptr& viewer){

  if(occ_cloud->points.size()){
    // process occ cloud
    vtkSmartPointer<vtkAppendPolyData> occ_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
    for (size_t i = 0; i < occ_cloud->points.size (); i++) {
      double x = occ_cloud->points[i].x;
      double y = occ_cloud->points[i].y;
      double z = occ_cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> occ_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      occ_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      occ_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      occ_append_filter->AddInput (occ_cube_source->GetOutput ());
#else
      occ_append_filter->AddInputData (occ_cube_source->GetOutput ());
#endif
    }

    // Remove duplicate points
    vtkSmartPointer<vtkCleanPolyData> occ_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New ();
    occ_clean_filter->SetInputConnection (occ_append_filter->GetOutputPort ());
    occ_clean_filter->Update ();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> occ_multi_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    occ_multi_mapper->SetInputConnection (occ_clean_filter->GetOutputPort ());
    vtkSmartPointer<vtkActor> occ_multi_actor = vtkSmartPointer<vtkActor>::New ();
    occ_multi_actor->SetMapper (occ_multi_mapper);
    occ_multi_actor->GetProperty ()->SetColor (1.0, 0.0, 0.0);
    occ_multi_actor->GetProperty ()->SetAmbient (1.0);
    occ_multi_actor->GetProperty ()->SetLineWidth (1);
    occ_multi_actor->GetProperty ()->EdgeVisibilityOn ();
    occ_multi_actor->GetProperty ()->SetOpacity (1.0);
    occ_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

    // Add the actor to the scene
    viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (occ_multi_actor);
  }

  if(fre_cloud->points.size()){
    // process fre cloud
    vtkSmartPointer<vtkAppendPolyData> fre_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
    for (size_t i = 0; i < fre_cloud->points.size (); i++) {
      double x = fre_cloud->points[i].x;
      double y = fre_cloud->points[i].y;
      double z = fre_cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> fre_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      fre_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      fre_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      fre_append_filter->AddInput (fre_cube_source->GetOutput ());
#else
      fre_append_filter->AddInputData (fre_cube_source->GetOutput ());
#endif
    }

    // Remove duplicate points
    vtkSmartPointer<vtkCleanPolyData> fre_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New ();
    fre_clean_filter->SetInputConnection (fre_append_filter->GetOutputPort ());
    fre_clean_filter->Update ();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> fre_multi_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    fre_multi_mapper->SetInputConnection (fre_clean_filter->GetOutputPort ());
    vtkSmartPointer<vtkActor> fre_multi_actor = vtkSmartPointer<vtkActor>::New ();
    fre_multi_actor->SetMapper (fre_multi_mapper);
    fre_multi_actor->GetProperty ()->SetColor (0.0, 1.0, 0.0);
    fre_multi_actor->GetProperty ()->SetAmbient (1.0);
    fre_multi_actor->GetProperty ()->SetLineWidth (1);
    fre_multi_actor->GetProperty ()->EdgeVisibilityOn ();
    fre_multi_actor->GetProperty ()->SetOpacity (1.0);
    fre_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

    // Add the actor to the scene
    viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (fre_multi_actor);
  }


  if(unn_cloud->points.size()){
    // process unn cloud
    vtkSmartPointer<vtkAppendPolyData> unn_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
    for (size_t i = 0; i < unn_cloud->points.size (); i++) {
      double x = unn_cloud->points[i].x;
      double y = unn_cloud->points[i].y;
      double z = unn_cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> unn_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      unn_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      unn_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      unn_append_filter->AddInput (unn_cube_source->GetOutput ());
#else
      unn_append_filter->AddInputData (unn_cube_source->GetOutput ());
#endif
    }

    // Remove duplicate points
    vtkSmartPointer<vtkCleanPolyData> unn_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New ();
    unn_clean_filter->SetInputConnection (unn_append_filter->GetOutputPort ());
    unn_clean_filter->Update ();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> unn_multi_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    unn_multi_mapper->SetInputConnection (unn_clean_filter->GetOutputPort ());
    vtkSmartPointer<vtkActor> unn_multi_actor = vtkSmartPointer<vtkActor>::New ();
    unn_multi_actor->SetMapper (unn_multi_mapper);
    unn_multi_actor->GetProperty ()->SetColor (1.0, 1.0, 0.0);
    unn_multi_actor->GetProperty ()->SetAmbient (1.0);
    unn_multi_actor->GetProperty ()->SetLineWidth (1);
    unn_multi_actor->GetProperty ()->EdgeVisibilityOn ();
    unn_multi_actor->GetProperty ()->SetOpacity (1.0);
    unn_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

    // Add the actor to the scene
    viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (unn_multi_actor);
  }

  // Render and interact
  viewer->getRenderWindow ()->Render ();
}
