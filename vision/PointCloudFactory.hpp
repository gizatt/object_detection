
#ifndef PointCloudFactory_H
#define PointCloudFactory_H

#include <lcm/lcm-cpp.hpp>
#include "kinect/depth_msg_t.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//factory to filter/downsample frames of depth images.
class PointCloudFactory
{
     //calibration params/ what to do to point cloud
     private:
          float voxeldx;
          float voxeldy;
          float voxeldz;

          //input from LCM/Kinect
          PointCloud::Ptr depthImageToPointCloud(const kinect::depth_msg_t* depthImage);

          PointCloud::Ptr voxelDownSample(PointCloud::Ptr cloud);


     public:
          PointCloudFactory();

          PointCloudFactory(const float dx, const float dy, const float dz);
          
          void ingestDepthImage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const kinect::depth_msg_t* depthImage);

};

#endif
