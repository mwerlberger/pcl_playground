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
   pcl::io::loadPLYFile<PointNormalT>(fname, *cloud_in);

   // split rgb and normals
   pcl::PointCloud<PointT>::Ptr cloud_rgb(new pcl::PointCloud<PointT>);
   pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>);
   pcl::copyPointCloud(*cloud_in, *cloud_rgb);
   pcl::copyPointCloud(*cloud_in, *cloud_normals);


   // display pointcloud + normals
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   viewer.setBackgroundColor (0.0, 0.0, 0.5);
   pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_rgb);
   viewer.addPointCloud<PointT> (cloud_rgb, rgb, "sample cloud");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   viewer.addPointCloudNormals<PointT, NormalT> (cloud_rgb, cloud_normals, 10, 0.05, "normals");
   viewer.addCoordinateSystem (0.3);
   viewer.initCameraParameters();

   while (!viewer.wasStopped ())
   {
     viewer.spinOnce ();
   }

   return EXIT_SUCCESS;
}
