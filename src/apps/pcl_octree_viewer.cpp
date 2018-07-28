#include <iostream>
#include <stdio.h>
#include <vector>

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
//void showCubes(double side,
//               const PointCloud::Ptr& occ_cloud,
//               const PointCloud::Ptr& fre_cloud,
//               const PointCloud::Ptr& unn_cloud,
//               Visualizer::Ptr& viewer);
void showCubes(double side,
               const PointCloud::Ptr& cloud,
               Visualizer::Ptr& viewer);

int main(int argc, char** argv){

  PointCloud::Ptr cloud (new PointCloud);
  //  PointCloud::Ptr occ_voxel_cloud (new PointCloud);
  //  PointCloud::Ptr fre_voxel_cloud (new PointCloud);
  //  PointCloud::Ptr unn_voxel_cloud (new PointCloud);

  Visualizer::Ptr viewer (new Visualizer ("Octree Viewer"));

  int depth;
  float point_size_=1.0;
  float resolution = 0.1;
  float inv_resolution = 1./resolution;

  std::cerr << "Loading file " << argv[1] << " ";
  pcl::io::loadPCDFile<Point> (argv[1], *cloud);
  std::cout << "with: " << cloud->points.size() << " points" << std::endl;

  Octree::Ptr octree(new Octree(resolution));
  octree->setInputCloud(cloud);
  octree->defineBoundingBox();
  octree->addPointsFromInputCloud();

  depth = static_cast<int> (octree->getTreeDepth());
  if (depth == 0)
    depth = 1;
  float s = std::sqrt (octree->getVoxelSquaredSideLen (depth)) / 2.0;

  std::cerr << "Octree: " << std::endl;
  std::cerr << ">>Depth: " << depth << std::endl;
  std::cerr << ">>Resolution: " << s*2 << std::endl;
  //  std::cerr << ">>Voxels: " << occ_voxel_cloud->size() << std::endl;

  Point min_pt,max_pt;
  pcl::getMinMax3D(*cloud,min_pt,max_pt);
  Eigen::Vector3f min(min_pt.x,min_pt.y,min_pt.z);
  Eigen::Vector3f max(max_pt.x,max_pt.y,max_pt.z);
  Eigen::Vector3i dim((int)((max.x()-min.x())*inv_resolution),
                      (int)((max.y()-min.y())*inv_resolution),
                      (int)((max.z()-min.z())*inv_resolution));

  std::cerr << ">>Min: " << min.transpose() << std::endl;
  std::cerr << ">>Max: " << max.transpose() << std::endl;
  std::cerr << ">>Dim: " << dim.transpose() << std::endl;

  std::cerr << std::endl << "Ray casting..." << std::endl;

  int occ=0,fre=0,unn=0;
  Eigen::Vector3f origin,end;
  Octree::AlignedPointTVector voxels;
  Point pt;

  std::vector<int> indices;
  std::vector<int> all_indices;
  PointCloud::Ptr occ_voxels (new PointCloud);
  PointCloud::Ptr unn_voxels (new PointCloud);
  for(int r=0; r<dim.z(); ++r)
    for(int c=0; c<dim.y(); ++c){
      origin.x() = min.x();
      origin.y() = min.y()+c*resolution;
      origin.z() = min.z()+r*resolution;
      end.x() = max.x();
      end.y() = min.y()+c*resolution;
      end.z() = min.z()+r*resolution;

      //      std::cerr << "Start: " << origin.transpose() << " - End: " << end.transpose() << std::endl;

      voxels.clear();
      octree->getApproxIntersectedVoxelCentersBySegment(origin, end, voxels, 0.5);

      //      std::cerr << "Traversed voxels: " << voxels.size() << std::endl;

      bool hit=false;
      for(int i =0;i<voxels.size();++i){

        //        std::cerr << "#" << i << ": ";

        pt.x = voxels[i].x;
        pt.y = voxels[i].y;
        pt.z = voxels[i].z;
        //        std::cerr << pt;

        indices.clear();
        bool found=octree->voxelSearch(pt,indices);
        if(!found){
          if(hit){
            //UNKNOWN
            //            unn_voxel_cloud->points.push_back(pt);
            pt.r = 0;
            pt.g = 1.0;
            pt.b = 0;
//            octree->addPointToCloud(pt,cloud);
            unn_voxels->points.push_back(pt);
            unn++;
            //            std::cerr << "UNKNOWN...";
            //            std::cin.get();
            //            std::cerr << std::endl;

            continue;
          }
          //FREE
          //          fre_voxel_cloud->points.push_back(pt);
          //          std::cerr << "FREE...";
          //          std::cin.get();
          //          std::cerr << std::endl;
          fre++;
          continue;
        }
        if(indices.size()){
          //OCCUPIED
          //          occ_voxel_cloud->points.push_back(pt);
          //          std::cerr << "OCCUPIED...";
          //          std::cerr << "voxel contains " << indices.size() << " points" << std::endl;

//          for(int idx : indices)
//            cloud->points.erase(cloud->begin()+idx);

          all_indices.insert(all_indices.end(),indices.begin(),indices.end());

//          octree->deleteVoxelAtPoint(pt);
          pt.r = 0;
          pt.g = 0;
          pt.b = 1.0;
//          octree->addPointToCloud(pt,cloud);
          occ_voxels->points.push_back(pt);

          if(!hit)
            hit=true;

          //          std::cin.get();
          //          std::cerr << std::endl;
          occ++;
          continue;
        }
      }
    }

  std::cerr << "Occupied: " << occ << std::endl;
  std::cerr << "Free    : " << fre << std::endl;
  std::cerr << "Unknown : " << unn << std::endl;

  for(int idx : all_indices)
    cloud->points.erase(cloud->begin()+idx);

  for(int i=0; i<occ_voxels->points.size(); ++i){
     octree->deleteVoxelAtPoint(occ_voxels->points[i]);
     octree->addPointToCloud(occ_voxels->points[i],cloud);
  }

  for(int i=0; i<unn_voxels->points.size(); ++i){
     octree->addPointToCloud(unn_voxels->points[i],cloud);
  }

  showCubes(s,cloud,viewer);

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
               const PointCloud::Ptr& cloud,
               Visualizer::Ptr& viewer){

  // process occ cloud
  vtkSmartPointer<vtkAppendPolyData> occ_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
  for (size_t i = 0; i < cloud->points.size (); i++) {

    if(cloud->points[i].b == 1){

      double x = cloud->points[i].x;
      double y = cloud->points[i].y;
      double z = cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> occ_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      occ_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      occ_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      occ_append_filter->AddInput (occ_cube_source->GetOutput ());
#else
      occ_append_filter->AddInputData (occ_cube_source->GetOutput ());
#endif
    }
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
  occ_multi_actor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
  occ_multi_actor->GetProperty ()->SetAmbient (1.0);
  occ_multi_actor->GetProperty ()->SetLineWidth (1);
  occ_multi_actor->GetProperty ()->EdgeVisibilityOn ();
  occ_multi_actor->GetProperty ()->SetOpacity (1.0);
  occ_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

  // Add the actor to the scene
  viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (occ_multi_actor);

  // process unn cloud
  vtkSmartPointer<vtkAppendPolyData> unn_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
  for (size_t i = 0; i < cloud->points.size (); i++) {

    if(cloud->points[i].g == 1){

      double x = cloud->points[i].x;
      double y = cloud->points[i].y;
      double z = cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> unn_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      unn_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      unn_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      unn_append_filter->AddInput (unn_cube_source->GetOutput ());
#else
      unn_append_filter->AddInputData (unn_cube_source->GetOutput ());
#endif
    }
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
  unn_multi_actor->GetProperty ()->SetColor (0.0, 1.0, 0.0);
  unn_multi_actor->GetProperty ()->SetAmbient (1.0);
  unn_multi_actor->GetProperty ()->SetLineWidth (1);
  unn_multi_actor->GetProperty ()->EdgeVisibilityOn ();
  unn_multi_actor->GetProperty ()->SetOpacity (1.0);
  unn_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

  // Add the actor to the scene
  viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (unn_multi_actor);

  // Render and interact
  viewer->getRenderWindow ()->Render ();
}
