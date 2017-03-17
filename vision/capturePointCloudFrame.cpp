#include <stdio.h>
#include <ctime>
#include <lcm/lcm-cpp.hpp>
#include "kinect/depth_msg_t.hpp"
#include "kinect/pointcloud_t.hpp"
#include <string>
#include <iostream>
#include "PointCloudHandler.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


int main(int argc, char *argv[])
{
    //LCM input

    lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    PointCloudHandler kinectCloudHandler;
     std::cout << argv[1] << std::endl;

    if (argc > 1)
    {
    	std::string arg1(argv[1]);
    	kinectCloudHandler = PointCloudHandler(arg1);
    }

    lcm.subscribe("DEPTH_IMAGE", &PointCloudHandler::ingestDepthImage, &kinectCloudHandler);
   	std::cout << "Starting to listen for depth images" << std::endl;

    while(0 == lcm.handle());

    // openNI input

     // PointCloudHandler asusCloudHandler;
     // pcl::Grabber* interface = new pcl::io::OpenNIGrabber();

     //   boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::Ptr&)> f =
     //     boost::bind (&PointCloudHandler::grabOpenNIImage, &asusCloudHandler, _1);

     //   interface->registerCallback(f);

     //   interface->start ();



    return 0;
}