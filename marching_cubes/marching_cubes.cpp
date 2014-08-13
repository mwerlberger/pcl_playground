#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal NormalT;
//typedef pcl::PointNormal PointNormalT;
typedef pcl::PointXYZRGBNormal PointNormalT;

//--------------------------------------------------------------------------------
// check if filename is a .ply file
bool checkFilename(std::string fname)
{
   std::string extension(".ply");
   if((fname.size() <= extension.size()) ||
         (fname.compare(fname.size() - extension.size(), extension.size(), extension) != 0) )
   {
      return false;
   }
   return true;
}

//--------------------------------------------------------------------------------
pcl::PolygonMesh::Ptr marchingcubesReconstruction(pcl::PointCloud<PointT>::Ptr cloud)
{
   std::cout << "entering marchingcubesReconstruction(cloud)" << std::endl;

   // point normals
   std::cout << "-- estimate point normals" << std::endl;
   pcl::NormalEstimationOMP<PointT, NormalT> n;
   n.setNumberOfThreads(8);

//   pcl::NormalEstimation<PointT, NormalT> n;
   pcl::PointCloud<NormalT>::Ptr normals(new pcl::PointCloud<NormalT>);
   pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
   tree->setInputCloud(cloud);
   n.setInputCloud(cloud);
   n.setSearchMethod(tree);
   n.setKSearch(20);
   n.compute(*normals);


   // points + normals in kdtree
   pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalT>);
   pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
   pcl::search::KdTree<PointNormalT>::Ptr tree2(new pcl::search::KdTree<PointNormalT>);
   tree2->setInputCloud(cloud_with_normals);

   pcl::io::savePLYFileBinary("cloud_with_normals.ply", *normals);

   // visualize normals
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   viewer.setBackgroundColor (0.0, 0.0, 0.5);
   pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
   viewer.addPointCloud<PointT> (cloud, rgb, "sample cloud");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   viewer.addPointCloudNormals<PointT, NormalT> (cloud, normals, 10, 0.05, "normals");
   viewer.addCoordinateSystem (0.3);
   viewer.initCameraParameters();

   while (!viewer.wasStopped ())
   {
     viewer.spinOnce ();
   }

   // marchingcubes surface fit
   std::cout << "-- marching cube" << std::endl;
   pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

   pcl::MarchingCubesHoppe<PointNormalT> mc;
   mc.setGridResolution(50.0, 50.0, 50.0);
   //mc.setIsoLevel(0.0);

   mc.setInputCloud(cloud_with_normals);
   mc.setSearchMethod(tree2);
   mc.reconstruct(*triangles);

   return triangles;
}

//--------------------------------------------------------------------------------
int main(int argc, char** argv)
{
   if(argc < 2)
   {
      PCL_ERROR("Syntax is: %s <source.ply>", argv[0]);
      return EXIT_FAILURE;
   }

   // load pointcloud from file
   std::string fname = std::string(argv[1]);
   if (!checkFilename(fname))
   {
      PCL_ERROR("Given file is not a valid .ply file: ", argv[1]);
      return EXIT_FAILURE;
   }
   std::cout << "loading " << fname << std::endl;
   pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
   pcl::io::loadPLYFile<PointT>(fname, *cloud_in);

   pcl::PolygonMesh::Ptr triangles = marchingcubesReconstruction(cloud_in);

   // sanity checks
   if (!triangles ||triangles->polygons.empty())
   {
      PCL_ERROR("Received an empty mesh....");
      return EXIT_FAILURE;
   }
   pcl::io::saveVTKFile("mesh_mc.vtk", *triangles);
   pcl::io::savePLYFileBinary("mesh_mc.ply", *triangles);


   return EXIT_SUCCESS;
}
