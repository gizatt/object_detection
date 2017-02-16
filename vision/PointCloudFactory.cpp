#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "PointCloudFactory.hpp"
  

  PointCloudFactory::PointCloudFactory() : voxeldx(0.1), voxeldy(0.1), voxeldz(0.1) {
  }

  PointCloudFactory::PointCloudFactory(const float dx, const float dy, const float dz) : voxeldx(dx), voxeldy(dy), voxeldz(dz){
  }

  pcl::PointCloud<pcl::PointXYZ> PointCloudFactory::depthImageToPointCloud(const kinect::depth_msg_t depthImage) {
    float fx = 525.0;  // focal length x
    float fy = 525.0; //focal length y
    float cx = 319.5; // optical center x
    float cy = 239.5;  // optical center y
    float factor = 5000;  //for the 16-bit PNG files

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize(depthImage.width*depthImage.height);
    int i;
    int j;
    for (i = 0; i < depthImage.height; i++){
      for(j = 0; j < depthImage.width; j++) {
        int index = depthImage.height * i + j;
        cloud.points[index].z = depthImage.depth_data[index]/factor;
        cloud.points[index].x = (i - cx) * cloud.points[index].z / fx;
        cloud.points[index].y = (j - cy) * cloud.points[index].z / fy;
      }
    }
    return cloud;
}

 pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFactory::voxelDownSample(pcl::PointCloud<pcl::PointXYZ> cloud){
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (cloud);

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(*cloud);
  filter.setLeafSize(voxeldx, voxeldy, voxeldz);
  filter.filter(*cloud_filtered);

  std::cerr << "PointCloud size after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
 
 return cloud_filtered;
}

  void PointCloudFactory::ingestDepthImage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const kinect::depth_msg_t* depthImage) {
    pcl::PointCloud<pcl::PointXYZ> cloud = depthImageToPointCloud(depthImage);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = voxelDownSample(cloud);

  }


