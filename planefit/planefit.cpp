#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


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

bool vectorSortFcn(int i, int j) { return (i<j); }


//--------------------------------------------------------------------------------
bool fitPlane(pcl::PointCloud<PointNormalT>::Ptr cloud,
              pcl::PointCloud<PointNormalT>::Ptr& plane,
              pcl::PointCloud<PointNormalT>::Ptr& objects)
{
   std::cout << "entering fitPlane(cloud)" << std::endl;
   plane.reset(new pcl::PointCloud<PointNormalT>());
   objects.reset(new pcl::PointCloud<PointNormalT>());

   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
   // Create the segmentation object
   pcl::SACSegmentation<PointNormalT> seg;
   // Optional
   seg.setOptimizeCoefficients(true);
   // Mandatory
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setDistanceThreshold(0.01);

   seg.setInputCloud(cloud);
   seg.segment(*inliers, *coefficients);

   std::cout << "Model coefficients: "
             << coefficients->values[0] << ", "
             << coefficients->values[1] << ", "
             << coefficients->values[2] << ", "
             << coefficients->values[3]
             << std::endl;

   if (inliers->indices.size () == 0)
   {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return false;
   }

   std::vector<int> inlierIndices = inliers->indices;
   std::sort(inlierIndices.begin(), inlierIndices.end(), vectorSortFcn);

   std::vector<int>::const_iterator inlierIdxIt = inlierIndices.begin();

   for (size_t i=0; i<cloud->size(); ++i)
   {
      if (i == *inlierIdxIt)
      {
         // plane
         plane->push_back(cloud->points[i]);
         ++inlierIdxIt;
      }
      else
      {
         objects->push_back(cloud->points[i]);
      }
   }

//   for (size_t i = 0; i<inliers->indices.size(); ++i)
//   {
//      plane->push_back(cloud->points[inliers->indices.at(i)]);
//   }

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
   pcl::io::loadPLYFile(fname, *cloud_in);

   // statistical outlier removal
   pcl::PointCloud<PointNormalT>::Ptr plane;
   pcl::PointCloud<PointNormalT>::Ptr objects;
   bool planefitSucces = fitPlane(cloud_in, plane, objects);

   if (!planefitSucces)
   {
      PCL_ERROR("planefit: something went wrong...");
      return EXIT_FAILURE;
   }

   // save output
   if (!plane ->empty())
   {
      pcl::io::savePLYFileBinary("cloud_planefit_plane.ply", *plane );
   }
   else
   {
      PCL_ERROR("planefit plane-pointcloud is empty.");
   }


   // save output
   if (!objects ->empty())
   {
      pcl::io::savePLYFileBinary("cloud_planefit_objects.ply", *objects );
   }
   else
   {
      PCL_ERROR("planefit objects-pointcloud is empty.");
   }

   return EXIT_SUCCESS;
}
