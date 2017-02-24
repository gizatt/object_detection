#include <iostream>
#include <stdint.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "PointCloudFactory.hpp" 
#include <pcl/visualization/cloud_viewer.h>
#include "kinect/pointcloud_t.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    

  PointCloudFactory::PointCloudFactory() {
  }

  static PointCloud::Ptr PointCloudFactory::depthImageToPointCloud(const kinect::depth_msg_t* depthImage) {
    
    int32_t frameSize = depthImage->width*depthImage->height;
    int centerX = (depthImage->width >> 1); 
    int centerY = (depthImage->height >> 1); 
    int index = 0;
 
    //pcl
    PointCloud::Ptr cloud = PointCloud::Ptr (new PointCloud);
    cloud->points.resize(frameSize);
    std::cout << frameSize << std::endl;
    cloud->height = depthImage->height;
    cloud->width = depthImage->width;
    //set ground coordinates
    cloud->sensor_origin_.setZero (); 
    cloud->sensor_orientation_.w () = 0.0f; 
    cloud->sensor_orientation_.x () = 1.0f; 
    cloud->sensor_orientation_.y () = 0.0f; 
    cloud->sensor_orientation_.z () = 0.0f; 

    //botcore
    kinect::pointcloud_t point_cloud;
    point_cloud.n_points = frameSize;
    point_cloud.points.resize(frameSize, std::vector<float>(3));
    point_cloud.utime = 0;
    point_cloud.seq = 0;
    point_cloud.frame_id = "0";
    point_cloud.n_channels = 1;
    point_cloud.channel_names.resize(1);
    point_cloud.channels.resize(1, std::vector<float>(frameSize));;

    //project 2D map into 3D space

    //constant from pcl docs
    float constant = 1.0f / 525;

    for (int i = -centerY; i < centerY; ++i) {
        for (int j = -centerX; j < centerX; ++j, ++index) {
        float z = depthImage->depth_data[index] * 0.001f;
        float x =  float(i) * z * constant;
        float y = float(j) * z * constant;

        //pcl cloud
        cloud->points[index].z = z;
        cloud->points[index].x = x;
        cloud->points[index].y = y;

        //bot core cloud
        point_cloud.points[index][0] = x;
        point_cloud.points[index][1] = y;
        point_cloud.points[index][2] = z;

      }
    }

   std::cout << "publishing KINECT_POINT_CLOUD_RAW" << std::endl;

    int i= messager.publish("KINECT_POINT_CLOUD_RAW", &point_cloud);
    std::cout << i << std::endl;

    return cloud;
}

 PointCloud::Ptr PointCloudFactory::voxelDownSample(PointCloud::Ptr cloud,float dx, float dy, float dz){
  
  PointCloud::Ptr cloud_filtered (new PointCloud);

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(dx, dy, dz);
  filter.filter(*cloud_filtered);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").\n";
       
  std::cerr << "PointCloud size after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";
 
 return cloud_filtered;
}

PointCloud::Ptr PointCloudFactory::zPassThroughFilter(PointCloud::Ptr cloud, float zLowerBound, float zUpperBound) {

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z_dir");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*cloud_filtered);
  return cloud_filtered
}



