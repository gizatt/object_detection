
#ifndef PointCloudFactory_H
#define PointCloudFactory_H
#include <string>
#include <lcm/lcm-cpp.hpp>
#include "kinect/depth_msg_t.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//factory to filter/downsample frames of depth images.
class PointCloudFactory
{
     //calibration params/ what to do to point cloud
     private:
          float voxeldx;
          float voxeldy;
          float voxeldz;

          //make static class for now
          PointCloudFactory();

    //filtering methods - maybe use some shape reconstruction algo to filter important point-talk to pete.      
     public:
          static PointCloud::Ptr depthImageToPointCloud(const kinect::depth_msg_t* depthImage);

          static PointCloud::Ptr voxelDownSample(PointCloud::Ptr cloud,float dx, float dy, float dz);
    
          static PointCloud::Ptr zPassThroughFilter(PointCloud::Ptr cloud, float zLowerBound, float zUpperBound);
};

#endif
