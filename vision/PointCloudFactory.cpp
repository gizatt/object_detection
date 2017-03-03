#include <iostream>
#include <stdint.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "PointCloudFactory.hpp" 
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    

PointCloudFactory::PointCloudFactory() {
}

PointCloud::Ptr PointCloudFactory::depthImageToPointCloud(const kinect::depth_msg_t* depthImage) {
    
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
    cloud->is_dense = false;

    //set ground coordinates
    cloud->sensor_origin_.setZero (); 
    cloud->sensor_orientation_.w () = 0.0f; 
    cloud->sensor_orientation_.x () = 1.0f; 
    cloud->sensor_orientation_.y () = 0.0f; 
    cloud->sensor_orientation_.z () = 0.0f; 

    //constant from pcl docs
    float constant = 1.0f / 525;
    
    //project 2D map into 3D space
    for (int i = -centerY; i < centerY; ++i) {
        for (int j = -centerX; j < centerX; ++j, ++index) {
        float z = depthImage->depth_data[index] * 0.001f;
        float x =  float(i) * z * constant;
        float y = float(j) * z * constant;

        //pcl cloud
        cloud->points[index].z = z;
        cloud->points[index].x = x;
        cloud->points[index].y = y;

      }
    }

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (zLowerBound, zUpperBound);
  pass.filter (*cloud_filtered);
  return cloud_filtered;
}

PointCloud::Ptr PointCloudFactory::statOutlierRemoval(PointCloud::Ptr cloud, int meanK, float stdDevMulThreshold) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud (cloud);
  filter.setMeanK (meanK);
  filter.setStddevMulThresh (stdDevMulThreshold);
  filter.filter (*cloud_filtered);
  return cloud_filtered;
}



