#ifndef PointCloudHandler_H

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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/*A class to handle point cloud I/O for an instance of a point cloud*/
 class PointCloudHandler 
 {

     private:
          PointCloud::Ptr currentPointCloud;
          // pcl::visualization::CloudViewer viewer("Cloud Viewer");     
          lcm::LCM messager;
          bool received_frame;
          string pcdOutputFile;


     public:

        //constructors
        PointCloudHandler(string pcdFileName);
        PointCloudHandler();

        //getters/setters
        void setCurrentPointCloud(PointCloud::Ptr cloud);
        const PointCloud::Ptr getCurrentPointCloud();

        //io
        void ingestDepthImage(const lcm::ReceiveBuffer* rbuf,const std::string& chan, const kinect::depth_msg_t* depthImage);
        void ingestDepthImages(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const kinect::depth_msg_t* depthImage);
        int readModelPCDFile(string pathToFile);
        int savePointCloud(PointCloud::Ptr cloud);
        void publishPointCloud(PointCloud::Ptr cloud);   

  };


    #endif
