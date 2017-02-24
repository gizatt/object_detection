// file: listener.cpp
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener listener.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o listener listener.cpp `pkg-config --cflags --libs lcm`

#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "kinect/depth_msg_t.hpp"
#include "PointCloudFactory.hpp"

 class PointCloudHandler 
 {
         public:
             ~PointCloudHandler() {}
     		
     		void ingestDepthImage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const kinect::depth_msg_t* depthImage) {
                printf("Received  depth message on channel \"%s\":\n", chan.c_str());
				PointCloud::Ptr cloud = depthImageToPointCloud(depthImage);
			}

			int readModelPCDFile(string pathToFile){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
				if (pcl::io::loadPCDFile<pcl::PointXYZ> (pathToFile, *cloud) == -1) {
   					PCL_ERROR ("Couldn't read pccd file \n");
    				return (-1);
  				}
  				std::cout << "Loaded " << pathToFile << " containing "
            	<< cloud->width * cloud->height << " points "<< std::endl;
			}

 			int savePointCloud(PointCloud::Ptr cloud, string name) {
     			string extension = ".pcd";
     			string fileName = name + extension;
     			string directory = "pclObjectLibrary";
     			if(pcl::io::savePCDFileASCII (directory + "name" + ".pcd",  *cloud){
        			std::cerr << "Saved " << cloud->points.size () << " data points to " << std::endl;
        			return 0;
      			}
    			else {
         			std::cerr << "Could not write to pcd file"<< std::endl;
         			return 1;
      			}

  			}
  			void publishPointCloud()
  	};



int main(int argc, char *argv[])
{
	std::cout << "Starting to listen for depth images" << std::endl;
    lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    PointCloudFactory factory;
    lcm.subscribe("DEPTH_IMAGE", &PointCloudFactory::ingestDepthImage, &factory);

    while(0 == lcm.handle());
    return 0;
}