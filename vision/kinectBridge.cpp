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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "kinect/pointcloud_t.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
static pcl::visualization::CloudViewer viewer("Cloud Viewer");

/*A class to handle point cloud I/O for an instance of a point cloud*/
 class PointCloudHandler 
 {
 		PointCloud::Ptr currentPointCloud;
 		//will need more than one viewer in the future
 		lcm::LCM messager;

        public:
        	//maybe init with some shared params
             ~PointCloudHandler() {
				viewer.runOnVisualizationThreadOnce (viewInit);
			}

     		///////////////////////////////////////////////////////////////
             //I/O
             /////////////////////////////////////////////////////////////

     		//for streams to PointCloundHandler use time filtering noise techniques
     		void ingestDepthImage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const kinect::depth_msg_t* depthImage) {

                printf("Received depth message on channel \"%s\":\n", chan.c_str());
				currentPointCloud = PointCloudFactory::depthImageToPointCloud(depthImage);
 				viewer.runOnVisualizationThread(viewCloud);
 				// publishPointCloud(currentPointCloud);
 			}

			int readModelPCDFile(string pathToFile){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
				if (pcl::io::loadPCDFile<pcl::PointXYZ> (pathToFile, *cloud) == -1) {
   					PCL_ERROR ("Couldn't read pcd file \n");
    				return (-1);
  				}
  				std::cout << "Loaded " << pathToFile << " containing "
            	<< cloud->width * cloud->height << " points "<< std::endl;
            	currentPointCloud = cloud;
			}

 			int savePointCloud(string name) {
     			string extension = ".pcd";
     			string fileName = name + extension;
     			string directory = "pclObjectLibrary";
     			if(pcl::io::savePCDFileASCII (directory + "name" + ".pcd",  *currentPointCloud)){
        			std::cerr << "Saved " << currentPointCloud->points.size () << " data points to " << std::endl;
        			return 0;
      			}
    			else {
         			std::cerr << "Could not write to pcd file"<< std::endl;
         			return 1;
      			}
      		}

  			void publishPointCloud(PointCloud::Ptr cloud){
  				//botcore lcm type
  				int32_t frameSize = cloud->points.size();
    			kinect::pointcloud_t point_cloud;
    			point_cloud.n_points = frameSize;
    			point_cloud.points.resize(frameSize, std::vector<float>(3));
    			point_cloud.utime = 0;
    			point_cloud.seq = 0;
    			point_cloud.frame_id = "0";
    			point_cloud.n_channels = 1;
    			point_cloud.channel_names.resize(1);
    			point_cloud.channels.resize(1, std::vector<float>(frameSize));
    			for(size_t i = 0; i < frameSize; ++i) {
    				cloud->points[i].x = currentPointCloud->points[i].x;
    				cloud->points[i].y = currentPointCloud->points[i].y;
    				cloud->points[i].z = currentPointCloud->points[i].z;
  				}

   				messager.publish("KINECT_POINT_CLOUD_RAW", &point_cloud);
   				std::cout << "publishing KINECT_POINT_CLOUD_RAW" << std::endl;
  			}

  			///////////////////////////////////////////////////////
  			//viz
  			/////////////////////////////////////////////////////
  			
  			static void viewInit (pcl::visualization::PCLVisualizer& viewer){
    			viewer.setBackgroundColor (1.0, 0.5, 1.0);
    		}
    		static void viewCloud (pcl::visualization::PCLVisualizer& viewer) {
    			std::stringstream ss;
				viewer.addText (ss.str(), 200, 300, "text", 0);
				viewer.updatePointCloud(currentPointCloud);
    		}
  	};



int main(int argc, char *argv[])
{
	std::cout << "Starting to listen for depth images" << std::endl;
    lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    //pass kinect point clouds into handler object
    PointCloudHandler kinectCloudHandler;
    lcm.subscribe("DEPTH_IMAGE", &PointCloudHandler::ingestDepthImage, &kinectCloudHandler);
    while(0 == lcm.handle());

    //pass pcl static point cloud into handler object
     // PointCloudHandler modelCloudHandler;
     // modelCloudHandler.readModelPcdFile();

    return 0;
}