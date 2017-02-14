#include <lcm/lcm-cpp.hpp>
#include "kinect/depth_msg_t.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//factory to filter/downsample frames of depth images.
class PointCloudFactory
{
     //calibration params/ what to do to point cloud
     private:
          float voxeldx;
          float voxeldy;
          float voxeldz;

     public:
          //make functional to more easily parallelize
          PointCloudFactory(const float dx = 0.1, const int dy  = 0.1, const int dz = 0.1);
          //input from LCM/Kinect
          pcl::PointCloud<pcl::PointXYZ> depthImageToPointCloud(const kinect::depth_msg_t depthImage);

          pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownSample(const pcl::PointCloud<pcl::PointXYZ>);

};
 