#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>

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
pcl::PolygonMesh::Ptr poissonReconstruction(pcl::PointCloud<PointNormalT>::Ptr cloud)
{
   std::cout << "entering poissonReconstruction(cloud)" << std::endl;

   // poisson surface fit
   pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());

   pcl::Poisson<PointNormalT> poisson;
   poisson.setDepth(7);
   poisson.setInputCloud(cloud);
   poisson.reconstruct(*triangles);

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
   pcl::PointCloud<PointNormalT>::Ptr cloud_in(new pcl::PointCloud<PointNormalT>);
   pcl::io::loadPLYFile(fname, *cloud_in);

   pcl::PolygonMesh::Ptr triangles = poissonReconstruction(cloud_in);

   // sanity checks
   if (!triangles ||triangles->polygons.empty())
   {
      PCL_ERROR("Received an empty mesh....");
      return EXIT_FAILURE;
   }
   pcl::io::saveVTKFile("mesh_poisson.vtk", *triangles);
   pcl::io::savePLYFileBinary("mesh_poisson.ply", *triangles);


   return EXIT_SUCCESS;
}
