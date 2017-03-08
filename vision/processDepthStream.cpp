#include <stdio.h>
#include <ctime>
#include <lcm/lcm-cpp.hpp>
#include "kinect/depth_msg_t.hpp"
#include "PointCloudFactory.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "kinect/pointcloud_t.hpp"
#include <string>
#include <iostream>
#include <fstream> 
#include "PointCloudHandler.hpp"
#include "PointCloudFactory.hpp"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


int main(int argc, char *argv[])
{

    lcm::LCM lcm;

    if(!lcm.good())
        return 1;
    PointCloudHandler kinectCloudHandler;
    lcm.subscribe("DEPTH_IMAGE", &PointCloudHandler::ingestDepthImages, &kinectCloudHandler);
   	std::cout << "Starting to listen for depth images" << std::endl;

    while(0 == lcm.handle());


    return 0;
}