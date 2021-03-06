#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>


typedef pcl::PointXYZRGB PointT;

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
pcl::PointCloud<PointT>::Ptr runMLS(pcl::PointCloud<PointT>::Ptr cloud)
{
   std::cout << "entering runMLS(cloud)" << std::endl;

   pcl::MovingLeastSquares<PointT, PointT> mls;
   mls.setInputCloud(cloud);
   mls.setSearchRadius(0.05);
   mls.setPolynomialFit(true);
   mls.setPolynomialOrder(2);
   mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
//   mls.setUpsamplingRadius(0.005);
//   mls.setUpsamplingStepSize(0.003);
   //mls.setPointDensity(10);
   mls.setDilationVoxelSize(0.0025);

   pcl::PointCloud<PointT>::Ptr cloud_mls(new pcl::PointCloud<PointT>());
   mls.process(*cloud_mls);
   return cloud_mls;
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

   // moving least squares
   pcl::PointCloud<PointT>::Ptr cloud_mls = runMLS(cloud_in);
   // save output
   if (!cloud_mls->empty())
   {
      pcl::io::savePLYFileBinary("cloud_xyzrgb_mls.ply", *cloud_mls);
   }
   else
   {
      PCL_ERROR("MLS result pointcloud is empty.");
      return EXIT_FAILURE;
   }

   return EXIT_SUCCESS;
}
