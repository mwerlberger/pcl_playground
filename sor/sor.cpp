#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>


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
pcl::PointCloud<PointT>::Ptr filterSOR(pcl::PointCloud<PointT>::Ptr cloud)
{
   std::cout << "entering filterSOR(cloud)" << std::endl;
   pcl::PointCloud<PointT>::Ptr cloud_sor(new pcl::PointCloud<PointT>());

   pcl::StatisticalOutlierRemoval<PointT> sor;
   sor.setInputCloud(cloud);
   sor.setMeanK(50);
   sor.setStddevMulThresh(1.0);
   sor.filter(*cloud_sor);

   return cloud_sor;
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

   // statistical outlier removal
   pcl::PointCloud<PointT>::Ptr cloud_sor = filterSOR(cloud_in);
   // save output
   if (!cloud_sor->empty())
   {
      pcl::io::savePLYFileBinary("cloud_xyzrgb_sor.ply", *cloud_sor);
   }
   else
   {
      PCL_ERROR("SOR filtered pointcloud is empty.");
      return EXIT_FAILURE;
   }

   return EXIT_SUCCESS;
}
