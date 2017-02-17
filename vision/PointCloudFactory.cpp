#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "PointCloudFactory.hpp" 
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

 static pcl::visualization::CloudViewer viewer ("cloud");


  PointCloudFactory::PointCloudFactory() : voxeldx(0.01), voxeldy(0.01), voxeldz(0.01) {
  }

  PointCloudFactory::PointCloudFactory(const float dx, const float dy, const float dz) : voxeldx(dx), voxeldy(dy), voxeldz(dz){
  }

  PointCloud::Ptr PointCloudFactory::depthImageToPointCloud(const kinect::depth_msg_t* depthImage) {
    float fx = 525.0;  // focal length x
    float fy = 525.0; //focal length y
    float cx = 319.5; // optical center x
    float cy = 239.5;  // optical center y
    float factor = 5000;  //for the 16-bit PNG files

    PointCloud::Ptr cloud = PointCloud::Ptr (new PointCloud);
    cloud->points.resize(depthImage->width*depthImage->height);
    std::cout << depthImage->width*depthImage->height << std::endl;
    cloud->height = depthImage->height;
    cloud->width = depthImage->width;

    int i;
    int j;
    for (i = 0; i < depthImage->height; i++){
      for(j = 0; j < depthImage->width; j++) {
        int index = depthImage->height * i + j;
        cloud->points[index].z = depthImage->depth_data[index]/factor;
        cloud->points[index].x = (i - cx) * cloud->points[index].z / fx;
        cloud->points[index].y = (j - cy) * cloud->points[index].z / fy;
      }
    }
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").\n";
    return cloud;
}

 PointCloud::Ptr PointCloudFactory::voxelDownSample(PointCloud::Ptr cloud){
  
  PointCloud::Ptr cloud_filtered (new PointCloud);

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(voxeldx, voxeldy, voxeldz);
  filter.filter(*cloud_filtered);


  std::cerr << "PointCloud size after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";
 
 return cloud_filtered;
}

  void PointCloudFactory::ingestDepthImage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const kinect::depth_msg_t* depthImage) {
   PointCloud::Ptr cloud = depthImageToPointCloud(depthImage);
   PointCloud::Ptr cloud_filtered = voxelDownSample(cloud);
   viewer.showCloud(cloud_filtered);

  }


