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


// register float constant = 1.0f / 525; 
// register int centerX = (pointcloud.width >> 1); 
// int centerY = (pointcloud.height >> 1); 
// register int depth_idx = 0; 
// for (int v = -centerY; v < centerY; ++v) 
// { 
//         for (register int u = -centerX; u < centerX; ++u, ++depth_idx) 
//         { 
//                 pcl::PointXYZ& pt = pointcloud.points[depth_idx]; 
//                 pt.z = depth_data[depth_idx] * 0.001f; 
//                 pt.x = static_cast<float> (u) * pt.z * constant; 
//                 pt.y = static_cast<float> (v) * pt.z * constant; 
//         } 
// } 
// pointcloud.sensor_origin_.setZero (); 
// pointcloud.sensor_orientation_.w () = 0.0f; 
// pointcloud.sensor_orientation_.x () = 1.0f; 
// pointcloud.sensor_orientation_.y () = 0.0f; 
// pointcloud.sensor_orientation_.z () = 0.0f; 

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
    cloud.sensor_origin_.setZero (); 
    cloud.sensor_orientation_.w () = 0.0f; 
    cloud.sensor_orientation_.x () = 1.0f; 
    cloud.sensor_orientation_.y () = 0.0f; 
    cloud.sensor_orientation_.z () = 0.0f; 
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

  void savePointCloud(PointCloud::Ptr cloud, string name) {
     string fileName = "name" + ".pcd";
     string directory = "pclObjectLibrary"
     pcl::io::savePCDFileASCII (directory + "name" + ".pcd", cloud);
     std::cerr << "Saved " << cloud.points.size () << " data points to "  + << std::endl;
  }



