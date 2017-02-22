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
      if (config["num_rays"])
        opt_num_rays_ = config["num_rays"].as<int>();

      // Load the model itself
      if (config["models"] == NULL){
        runtime_error("Must specify models for point cloud generator to work with.");
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

    pcl::PointCloud<PointType>::Ptr samplePointCloudFromSurface( ){
      KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_);

      Matrix3Xd all_vertices;
      DrakeShapes::TrianglesVector all_faces;
      vector<double> face_cumulative_area = {0.0};

      // Collect faces from all bodies in this configuration
      for (auto iter = robot_.bodies.begin(); iter != robot_.bodies.end(); iter++) {
        auto collision_elems = (*iter)->get_collision_element_ids();
        for (auto collision_elem = collision_elems.begin();
                  collision_elem != collision_elems.end();
                  collision_elem++) {
          auto element = robot_.FindCollisionElement(*collision_elem);
          if (element->hasGeometry()){
            const DrakeShapes::Geometry & geometry = element->getGeometry();
            if (geometry.hasFaces()){
              Matrix3Xd points;
              geometry.getPoints(points);
              // Transform them to this robot's frame
              points = robot_.transformPoints(robot_kinematics_cache, points, 0, (*iter)->get_body_index());

              all_vertices.conservativeResize(3, points.cols() + all_vertices.cols());
              all_vertices.block(0, all_vertices.cols() - points.cols(), 3, points.cols()) = points;
              DrakeShapes::TrianglesVector faces;
              geometry.getFaces(faces);
              // Calculate the face surface area, so we can do even sampling from the surface.
              // See http://math.stackexchange.com/a/128999.
              // Also use this loop to offset the vertex indices to the appropriate all_vertices indices.
              for (auto face = faces.begin(); face != faces.end(); face++) {
                Vector3d a = points.col( (*face)[0] );
                Vector3d b = points.col( (*face)[1] );
                Vector3d c = points.col( (*face)[2] );
                double area = ((b - a).cross(c - a)).norm() / 2.;
                face_cumulative_area.push_back(face_cumulative_area[face_cumulative_area.size()-1] + area);

                (*face)[0] += (all_vertices.cols() - points.cols());
                (*face)[1] += (all_vertices.cols() - points.cols());
                (*face)[2] += (all_vertices.cols() - points.cols());
              }
              all_faces.insert(all_faces.end(), faces.begin(), faces.end());
            } 
          }
        }
      }

      // Normalize cumulative areas.
      for (int i=0; i<face_cumulative_area.size(); i++){
        face_cumulative_area[i] /= face_cumulative_area[face_cumulative_area.size() - 1];
      }

      pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType> ());

      while (pc->size() < opt_num_rays_){
        // Pick the face we'll sample from
        double sample = randrange(1E-12, 1.0 - 1E-12);
        int k = 0;
        for (k=0; k<face_cumulative_area.size(); k++){
          if (face_cumulative_area[k] >= sample){
            break;
          }
        }
        k -= 1;

        Vector3d a = all_vertices.col(all_faces[k][0]);
        Vector3d b = all_vertices.col(all_faces[k][1]);
        Vector3d c = all_vertices.col(all_faces[k][2]);

        double s1 = randrange(0.0, 1.0); 
        double s2 = randrange(0.0, 1.0);

        if (s1 + s2 <= 1.0){
          Vector3d pt = a + 
                        s1 * (b - a) +
                        s2 * (c - a);
          pc->push_back(Vector3dToPoint(pt));
        }
      }

      return pc;
    }

  private:
    int opt_scene_sampling_mode_ = 0;
    int opt_num_rays_ = 100;
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
    runtime_error("Config needs a point cloud generator option set.");
  }
  PointCloudGenerator pcg(config["point_cloud_generator_options"]);

  // Generate a point set
  pcl::PointCloud<PointType>::Ptr model_pts = pcg.samplePointCloudFromSurface();


  // And set up our detector
  if (config["detector_options"] == NULL){
    runtime_error("Config needs a detector option set.");
  }
  MILPMultipleMeshModelDetector detector(config["detector_options"]);


  // Visualize the generated model point cloud
  pcl::visualization::PCLVisualizer viewer ("Point Collection");

  pcl::visualization::PointCloudColorHandlerCustom<PointType> model_color_handler (model_pts, 128, 255, 255);
  viewer.addPointCloud<PointType>(model_pts, model_color_handler, "model pts"); 
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model pts");

  while (!viewer.wasStopped ())
    viewer.spinOnce ();

  return 0;
}
