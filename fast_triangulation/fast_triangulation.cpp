#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

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
pcl::PolygonMesh::Ptr fastTriangulation(pcl::PointCloud<PointT>::Ptr cloud)
{
   std::cout << "entering fastTriangulation(cloud)" << std::endl;

   // point normals
   std::cout << "-- estimate point normals" << std::endl;
   pcl::NormalEstimation<PointT, NormalT> n;
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

   // gp triangulation
   pcl::GreedyProjectionTriangulation<PointNormalT> gp3;
   pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());
   gp3.setSearchRadius(0.1);
   gp3.setMu(3);
   gp3.setMaximumNearestNeighbors(100);
   gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
   gp3.setMinimumAngle(M_PI/18); // 10 degrees
   gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
   gp3.setNormalConsistency(false);

   gp3.setInputCloud(cloud_with_normals);
   gp3.setSearchMethod(tree2);
   gp3.reconstruct(*triangles);

////   // Additional vertex information
////   std::vector<int> parts = gp3.getPartIDs();
////   std::vector<int> states = gp3.getPointStates();

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
   pcl::io::loadPLYFile(fname, *cloud_in);

   pcl::PolygonMesh::Ptr triangles = fastTriangulation(cloud_in);

   // sanity checks
   if (!triangles ||triangles->polygons.empty())
   {
      PCL_ERROR("Received an empty mesh....");
      return EXIT_FAILURE;
   }
   pcl::io::saveVTKFile("mesh_gp3.vtk", *triangles);


   return EXIT_SUCCESS;
}
