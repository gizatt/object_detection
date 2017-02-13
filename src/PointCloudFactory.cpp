#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudFactory {
  
  PointCloudFactory::PointCloudFactory(const float dx = 0.1, const int dy  = 0.1, const int dz = 0.1) {
    floatdx = dx
    floatdy = dy
    floatdz = dz
  }

  pcl::PointCloud<pcl::PointXYZ> PointCloudFactory::depthImageToPointCloud(const kinect::depth_msg_t depthImage) {
    float fx = 525.0;  // focal length x
    float fy = 525.0; //focal length y
    float cx = 319.5; // optical center x
    float cy = 239.5;  // optical center y

    float factor = 5000  //for the 16-bit PNG files

    pcl::PointCloud<pcl::PointXYZ> cloud;
    clound.points.resize(depthImage.width*depthImage.height)
    for (int i = 0; i < depth_image.height i++){
      for(int j = 0; j < depth_image.width j++) {
        int index = depth.height * i + j;
        cloud.points[index].x = depthImage.depth_data[index] /factor;
        cloud.points[index].y = (u - cx) * Z / fx;
        cloud.points[index].z = (v - cy) * Z / fy;
      }
    }

}

 pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownSample(const pcl::PointCloud<pcl::PointXYZ>){
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<PointXYZ>> filter;
  sor.setLeafSize(floatdx, floatdy, floatdz);
  filter.applyFilter(*cloud_filtered);

  std::cerr << "PointCloud size after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
 }
 return cloud_filtered;


}