/*
 */

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/rotation_constraint.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

#include "yaml-cpp/yaml.h"
#include "common/common.hpp"
#include "optimization_helpers.h"
#include "rotation_helpers.h"
#include "pcl_helpers.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

class PointCloudGenerator {
  public:
    PointCloudGenerator(YAML::Node config){
      if (config["scene_sampling_mode"])
        opt_scene_sampling_mode_ = config["scene_sampling_mode"].as<int>();
      if (config["sample_distance"])
        opt_sample_distance_ = config["sample_distance"].as<double>();

      // Load the model itself
      if (config["models"] == NULL){
        std::runtime_error("Must specify models for point cloud generator to work with.");
      }
      // Model will be a RigidBodyTree.
      q_robot_.resize(0);
      int old_q_robot_size = 0;
      for (auto iter=config["models"].begin(); iter!=config["models"].end(); iter++){
        string urdf = (*iter)["urdf"].as<string>();
        AddModelInstanceFromUrdfFileWithRpyJointToWorld(urdf, &robot_);
        // And add initial state info that we were passed
        vector<double> q0 = (*iter)["q0"].as<vector<double>>();
        assert(robot_.get_num_positions() - old_q_robot_size == q0.size());
        q_robot_.conservativeResize(robot_.get_num_positions());
        for (int i=0; i<q0.size(); i++){
          q_robot_[old_q_robot_size] = q0[i];
          old_q_robot_size++; 
        }
      }
    }

    pcl::PointCloud<PointType>::Ptr generatePointCloud(){
      pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType> ());

      printf("This is next!\n");

      return pc;
    }

  private:
    int opt_scene_sampling_mode_ = 0;
    double opt_sample_distance_ = 0.05;

    RigidBodyTree<double> robot_;
    VectorXd q_robot_;
};

class MILPMultipleMeshModelDetector {
  public:
    MILPMultipleMeshModelDetector(YAML::Node config){
      printf("TODO in here\n");
    }
};



int main(int argc, char** argv) {
  srand(getUnixTime());

  if (argc != 2){
    printf("Use: milp_multiple_mesh_models_detector_drake <config file>\n");
    exit(-1);
  }

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "MILP Multiple Mesh Models Detector, Drake Ver, " << asctime(curtime);
  cout << "Config file " << string(argv[1]) << endl;
  cout << "***************************" << endl << endl;


  // Bring in config file
  string yamlString = string(argv[1]);
  YAML::Node config = YAML::LoadFile(yamlString);

  // Set up a point cloud generator
  if (config["point_cloud_generator_options"] == NULL){
    std::runtime_error("Config needs a point cloud generator option set.");
  }
  PointCloudGenerator pcg(config["point_cloud_generator_options"]);

  // Generate a point set
  pcl::PointCloud<PointType>::Ptr pc = pcg.generatePointCloud();



  // And set up our detector
  if (config["detector_options"] == NULL){
    std::runtime_error("Config needs a detector option set.");
  }
  MILPMultipleMeshModelDetector detector(config["detector_options"]);



  return 0;
}
