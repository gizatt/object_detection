/*
 */

#include <string>
#include <stdexcept>
#include <iostream>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/rotation_constraint.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

#include <lcm/lcm-cpp.hpp>

#include "yaml-cpp/yaml.h"
#include "common/common.hpp"
#include "unistd.h"

#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "optimization_helpers.h"
#include "rotation_helpers.h"
#include "pcl_helpers.h"

using namespace std;
using namespace Eigen;
using namespace drake::solvers;
using namespace drake::parsers::urdf;

const double kBigNumber = 100.0; // must be bigger than largest possible correspondance distance

struct Model {
  std::string name;
  bool matchable = true;
  Eigen::Affine3d scene_transform;
  Eigen::Affine3d model_transform;
  Eigen::Matrix3Xd vertices;
  std::vector< std::vector<int> > faces; // each face is a list of vertex indices, clockwise around the face
};

Model load_model_from_yaml_node(YAML::Node model_node){
  Model m;
  m.name = model_node["name"].as<string>();
  printf("Loading model %s...\n", m.name.c_str());
  int num_verts = model_node["vertices"].size();
  printf("\twith %d verts\n", num_verts);
  m.vertices.resize(3, num_verts);
  std::map<string, int> vertex_name_to_index;
  int k=0;
  for (auto iter = model_node["vertices"].begin(); iter != model_node["vertices"].end(); iter++){
    string name = iter->first.as<string>();
    vertex_name_to_index[name] = k;
    auto pt = iter->second.as<vector<double>>();
    for (int i=0; i<3; i++)
      m.vertices(i, k) = pt[i];
    k++;
  }

  printf("\twith %ld faces\n", model_node["faces"].size());
  for (auto iter = model_node["faces"].begin(); iter != model_node["faces"].end(); iter++){
    auto face_str = iter->second.as<vector<string>>();
    vector<int> face_int;
    for (int i=0; i<face_str.size(); i++){
      face_int.push_back(vertex_name_to_index[face_str[i]]);
    }
    m.faces.push_back(face_int);
  }

  vector<double> scene_tf = model_node["scene_tf"].as<vector<double>>();
  m.scene_transform.setIdentity();
  m.scene_transform.translation() = Vector3d(scene_tf[0], scene_tf[1], scene_tf[2]);
  // todo: ordering, or really just move to quat
  m.scene_transform.rotate (Eigen::AngleAxisd (scene_tf[5], Eigen::Vector3d::UnitZ()));
  m.scene_transform.rotate (Eigen::AngleAxisd (scene_tf[4], Eigen::Vector3d::UnitY()));
  m.scene_transform.rotate (Eigen::AngleAxisd (scene_tf[3], Eigen::Vector3d::UnitX()));

  vector<double> model_tf = model_node["model_tf"].as<vector<double>>();
  m.model_transform.setIdentity();
  m.model_transform.translation() = Vector3d(model_tf[0], model_tf[1], model_tf[2]);
  // todo: ordering, or really just move to quat
  m.model_transform.rotate (Eigen::AngleAxisd (model_tf[5], Eigen::Vector3d::UnitZ()));
  m.model_transform.rotate (Eigen::AngleAxisd (model_tf[4], Eigen::Vector3d::UnitY()));
  m.model_transform.rotate (Eigen::AngleAxisd (model_tf[3], Eigen::Vector3d::UnitX()));

  return m;
}
Eigen::Matrix3Xd get_face_midpoints(Model m){
  Matrix3Xd out(3, m.faces.size());
  for (int i=0; i<m.faces.size(); i++){
    double w0 = 0.3333;
    double w1 = 0.3333;
    double w2 = 0.3333;
    out.col(i) = w0*m.vertices.col(m.faces[i][0]) +  
                 w1*m.vertices.col(m.faces[i][1]) + 
                 w2*m.vertices.col(m.faces[i][2]);
  }
  return out;
}
Eigen::Matrix3Xd sample_from_surface_of_model(Model m, int N){
  Matrix3Xd out(3, N);
  for (int i=0; i<N; i++){
    int face = rand() % m.faces.size(); // assume < RAND_MAX faces...
    double w0 = randrange(0., 1.0);
    double w1 = randrange(0., 1.0);
    double w2 = randrange(0., 1.0);
    double tot = w0 + w1 + w2;
    w0 /= tot; w1 /= tot; w2 /= tot;
    out.col(i) = w0*m.vertices.col(m.faces[face][0]) +  
                 w1*m.vertices.col(m.faces[face][1]) + 
                 w2*m.vertices.col(m.faces[face][2]);
  }
  return out;
}

bool pending_redraw = true;
int draw_mode = 0;
bool reextract_solution = true;
int target_sol = 0;
int max_num_sols = 1;
int target_corresp_id = 0;
int max_corresp_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "z" && event.keyDown ())
  { 
    draw_mode = (draw_mode + 1) % 3;
  }
  else if (event.getKeySym() == "Right" && event.keyDown()) {
    target_corresp_id = (target_corresp_id + 1) % max_corresp_id;
  }
  else if (event.getKeySym() == "Left" && event.keyDown()) {
    target_corresp_id = (target_corresp_id - 1 + max_corresp_id) % max_corresp_id;
  }
  else if (event.getKeySym() == "Up" && event.keyDown()) {
    target_sol = (target_sol + 1) % max_num_sols;
    reextract_solution = true;
  }
  else if (event.getKeySym() == "Down" && event.keyDown()) {
    target_sol = (target_sol - 1 + max_num_sols) % max_num_sols;
    reextract_solution = true;
  }
  pending_redraw = true;
}

int main(int argc, char** argv) {
  srand(getUnixTime());

  int optNumPointQuads = 10;
  int optNumRays = 10;
  int optSceneSamplingMode = 0;
  int optSampleModel = -1;
  double optSampleDistance = 10.0;
  int optNumHODBins = 5;
  double optMaxHODDist = 5;

  if (argc != 2){
    printf("Use: miqp_histogram_of_distances_detector <config file>\n");
    exit(-1);
  }

  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  if (!lcm->good()) {
    throw std::runtime_error("LCM is not good");
  }

  pcl::visualization::PCLVisualizer viewer ("Point Collection");

  // Set up robot
  string yamlString = string(argv[1]);
  YAML::Node modelsNode = YAML::LoadFile(yamlString);
  if (modelsNode["options"]["num_point_quads"])
    optNumPointQuads = modelsNode["options"]["num_point_quads"].as<int>();
  if (modelsNode["options"]["num_rays"])
    optNumRays = modelsNode["options"]["num_rays"].as<int>();
  if (modelsNode["options"]["scene_sampling_mode"])
    optSceneSamplingMode = modelsNode["options"]["scene_sampling_mode"].as<int>();
  if (modelsNode["options"]["sample_distance"])
    optSampleDistance = modelsNode["options"]["sample_distance"].as<double>();
  if (modelsNode["options"]["sample_model"])
    optSampleModel = modelsNode["options"]["sample_model"].as<int>();
  if (modelsNode["options"]["num_hod_bins"])
    optNumHODBins = modelsNode["options"]["num_hod_bins"].as<int>();
  if (modelsNode["options"]["max_hod_dist"])
    optMaxHODDist = modelsNode["options"]["max_hod_dist"].as<double>();


  std::vector<Model> models;

  printf("Loaded\n");
  for (auto iter=modelsNode["models"].begin(); iter != modelsNode["models"].end(); iter++){
    models.push_back(load_model_from_yaml_node(*iter));
  }
  if (models.size() != 1){
    printf("ONLY SUPPORTING 1 MODEL FOR NOW\n");
    return -1;
  }
  printf("Parsed models\n");

  // Render scene cloud by sampling surface of objects
  pcl::PointCloud<PointType>::Ptr scene_pts (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr actual_model_pts (new pcl::PointCloud<PointType> ());
  map<int, int> correspondences_gt;

  // different kind of sampling: pick random vertex
  int model_num = rand() % models.size();

  int k=0;
  for (int m=0; m<models.size(); m++){
    if (model_num == -1 || m == model_num){
      int vertex_num = rand() % models[m].vertices.cols();
      Vector3d target_vertex = models[m].vertices.col(vertex_num);
      int num_scene_samples = 0;
      int target_num_rays = optNumRays;
      if (model_num == -1) target_num_rays /= models.size();
      Matrix3Xd scene_samples;
      while (num_scene_samples < target_num_rays){
        Matrix3Xd new_scene_samples = sample_from_surface_of_model(models[m], target_num_rays);
        scene_samples.conservativeResize(3, num_scene_samples + new_scene_samples.cols());
        for (int i=0; i<new_scene_samples.cols(); i++){
          if (optSceneSamplingMode != 2 || (target_vertex - new_scene_samples.col(i)).norm() < optSampleDistance){
            scene_samples.col(num_scene_samples) = new_scene_samples.col(i);
            num_scene_samples++;
          }
          if (num_scene_samples == optNumRays){
            break;
          }
        }
        scene_samples.conservativeResize(3, num_scene_samples);
      }

      auto samples_model_frame = models[m].model_transform * scene_samples;
      auto samples_scene_frame = models[m].scene_transform * scene_samples;
      for (int i=0; i<samples_scene_frame.cols(); i++){
        scene_pts->push_back(PointType( samples_scene_frame(0, i), samples_scene_frame(1, i), samples_scene_frame(2, i)));
        // overkill doing this right now, as the scene = simpletf * model in same order. but down the road may get
        // more imaginative with this
        actual_model_pts->push_back(PointType( samples_model_frame(0, i), samples_model_frame(1, i), samples_model_frame(2, i)));
        correspondences_gt[k] = k;
        k++;
      }
    }
  }
  printf("Running with %d scene pts and %d model points\n", (int)scene_pts->size(), (int)actual_model_pts->size());

  Matrix3Xd model_pts(3, actual_model_pts->size());
  for (int i=0; i<actual_model_pts->size(); i++){
    model_pts.col(i) = Vector3d( actual_model_pts->at(i).x, actual_model_pts->at(i).y, actual_model_pts->at(i).z);
  }

  // Generate histogram of distances for each point in the scene, and each point in the model
  MatrixXd HOD_Scene(scene_pts->size(), optNumHODBins);
  HOD_Scene.setZero();
  MatrixXd HOD_Model(actual_model_pts->size(), optNumHODBins);
  HOD_Model.setZero();
  VectorXd HOD_Bounds(optNumHODBins+1);
  for (int i=0; i<optNumHODBins+1; i++){
    HOD_Bounds(i) = ((double)i) * (optMaxHODDist / (double)optNumHODBins);
  }
  cout << "********************" << endl;
  cout << "HOD Bounds: " << endl;
  cout << HOD_Bounds << endl;
  cout << "*******************" << endl;
  // scene points
  for (int i=0; i<scene_pts->size(); i++){
    int total_num = 0;
    Vector3d pt = pointToVector3d(scene_pts->at(i));
    for (int j=0; j<scene_pts->size(); j++){
      if (i != j){
        double dist = (pt - pointToVector3d(scene_pts->at(j))).norm();
        for (int k=0; k<optNumHODBins; k++){
          if (dist >= HOD_Bounds(k) && dist < HOD_Bounds(k+1)){
            HOD_Scene(i, k) += 1;
            total_num++;
            break;
          }
        }
      }
    }
    for (int k=0; k<optNumHODBins; k++){
      HOD_Scene(i, k) /= (double)total_num;
    }
  }
  // model points
  for (int i=0; i<model_pts.cols(); i++){
    int total_num = 0;
    Vector3d pt = model_pts.col(i);
    for (int j=0; j<model_pts.cols(); j++){
      if (i != j){
        double dist = (pt - model_pts.col(j)).norm();
        for (int k=0; k<optNumHODBins; k++){
          if (dist >= HOD_Bounds(k) && dist < HOD_Bounds(k+1)){
            HOD_Model(i, k) += 1;
            total_num++;
            break;
          }
        }
      }
    }
    for (int k=0; k<optNumHODBins; k++){
      HOD_Model(i, k) /= (double)total_num;
    }
  }
  cout << "********************" << endl;
  cout << "Model HODS: " << endl;
  cout << HOD_Model << endl;
  cout << "*******************" << endl;
  cout << "********************" << endl;
  cout << "Scene HODS: " << endl;
  cout << HOD_Scene << endl;
  cout << "*******************" << endl;


  // See https://www.sharelatex.com/project/5850590c38884b7c6f6aedd1
  // for problem formulation
  MathematicalProgram prog;
  
  // Each row allows selection of model point to correspond with given scene point
  auto C = prog.NewBinaryVariables(scene_pts->size(), actual_model_pts->size(), "C");

  // Constrain each row of C to sum to at most 1, so we correspond scene points to at 
  // most one model point
  Eigen::MatrixXd C1 = Eigen::MatrixXd::Ones(1, C.cols());
  for (size_t k=0; k<C.rows(); k++){
    prog.AddLinearConstraint(C1, 0, 1., C.row(k).transpose());
  }

  // Force all elems of C in [0, 1], to be thorough. Might help with relaxation, though
  // I'd guess this comes in automatically during the solve?
  for (int i=0; i<C.rows(); i++){
    for (int j=0; j<C.cols(); j++){
      prog.AddBoundingBoxConstraint(0.0, 1.0, C(i, j));
    }
  }

  printf("Starting to add correspondence costs...\n");
  for (int i=0; i<scene_pts->size(); i++){
    printf("\r");
    printf("\t(%d)/(%d)", i+1, (int)scene_pts->size());
    //  want to find
    // min || HOD_Scene(i, :)^T - HOD_Model^T * C(i, :)^T ||^2
    // expanded out, this is, for B = HOD_Scene(i, :)^T, A = HOD_Model^T, x = C(i, :)^T,
    // || -Ax + B || ^2
    for (int k=0; k<optNumHODBins; k++){
      prog.AddL2NormCost( HOD_Model.col(k).transpose(), HOD_Scene.block<1, 1>(i, k), C.row(i).transpose() );
    }
    
  }
  printf("\n");

  double now = getUnixTime();

  GurobiSolver gurobi_solver;
  MosekSolver mosek_solver;

  prog.SetSolverOption(SolverType::kGurobi, "OutputFlag", 1);
  prog.SetSolverOption(SolverType::kGurobi, "LogToConsole", 1);
  prog.SetSolverOption(SolverType::kGurobi, "LogFile", "loggg.gur");
  prog.SetSolverOption(SolverType::kGurobi, "DisplayInterval", 5);

  if (modelsNode["options"]["gurobi_int_options"]){
    for (auto iter = modelsNode["options"]["gurobi_int_options"].begin();
         iter != modelsNode["options"]["gurobi_int_options"].end();
         iter++){
      prog.SetSolverOption(SolverType::kGurobi, iter->first.as<string>(), iter->second.as<int>());
    }
  }
  if (modelsNode["options"]["gurobi_float_options"]){
    for (auto iter = modelsNode["options"]["gurobi_float_options"].begin();
         iter != modelsNode["options"]["gurobi_float_options"].end();
         iter++){
      prog.SetSolverOption(SolverType::kGurobi, iter->first.as<string>(), iter->second.as<float>());
    }
  }

//  prog.SetSolverOption(SolverType::kGurobi, "Cutoff", 50.0);
// isn't doing anything... not invoking this tool right?
//  prog.SetSolverOption(SolverType::kGurobi, "TuneJobs", 8);
//  prog.SetSolverOption(SolverType::kGurobi, "TuneResults", 3);
  //prog.SetSolverOption(SolverType::kGurobi, )


  auto out = gurobi_solver.Solve(prog);
  string problem_string = "rigidtf";
  double elapsed = getUnixTime() - now;

  max_num_sols = prog.get_num_solutions();

  //prog.PrintSolution();
  printf("Code %d, problem %s solved for %lu scene solved in: %f\n", out, problem_string.c_str(), scene_pts->size(), elapsed);

  // Extract into a set of correspondences
  struct PointCorrespondence {
    PointType model_pt;
    PointType scene_pt;
    int model_pt_ind;
    int scene_pt_ind;
  };

  struct ObjectDetection {
    Eigen::Affine3f est_tf;
    std::vector<PointCorrespondence> correspondences;
    int obj_ind;
  };

  // Viewer main loop
  // Pressing left-right arrow keys allows viewing of individual point-face correspondences
  // Pressing "z" toggles viewing everything at once or doing individual-correspondence viewing
  // Pressing up-down arrow keys scrolls through different optimal solutions (TODO(gizatt) make this happen)
  viewer.setShowFPS(false);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler (scene_pts, 255, 255, 128);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> model_color_handler (actual_model_pts, 255, 255, 128);
  viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

  max_corresp_id = scene_pts->size();
  pending_redraw = true;

  std::vector<ObjectDetection> detections;
  MatrixXd C_est;

  reextract_solution = true;
  double objective = -12345;

  while (!viewer.wasStopped ()){
    if (reextract_solution){
      detections.clear();

      C_est = prog.GetSolution(C, target_sol);

      for (int i=0; i<models.size(); i++){
        ObjectDetection detection;
        detection.obj_ind = i;

        printf("************************************************\n");
        printf("Concerning model %d (%s):\n", i, models[i].name.c_str());
        printf("------------------------------------------------\n");
        printf("Ground truth TF: ");
        auto ground_truth_tf = models[i].scene_transform.inverse().cast<float>() 
                              * models[i].model_transform.cast<float>();
        cout << ground_truth_tf.translation().transpose() << endl;
        cout << ground_truth_tf.matrix().block<3,3>(0,0) << endl;
        printf("------------------------------------------------\n");

        detection.est_tf.setIdentity();
//        detection.est_tf.translation() = Tf;
//        detection.est_tf.matrix().block<3,3>(0,0) = Rf;

        for (int scene_i=0; scene_i<scene_pts->size(); scene_i++){
          PointCorrespondence new_corresp;
          new_corresp.scene_pt_ind = scene_i;
          new_corresp.scene_pt = scene_pts->at(scene_i);
          for (int model_pt_j=0; model_pt_j<model_pts.cols(); model_pt_j++){
            if (C_est(scene_i, model_pt_j) > 0.5){
                new_corresp.model_pt = actual_model_pts->at(model_pt_j);
                new_corresp.model_pt_ind = model_pt_j;
                break;
            }
          }
          detection.correspondences.push_back(new_corresp);
        }
        if (detection.correspondences.size() > 0)
          detections.push_back(detection);
      }
      reextract_solution = false;

      //objective = prog.GetSolution(phi).sum() + prog.GetSolution(alpha).sum();
    }

    if (pending_redraw){
      viewer.removeAllPointClouds();
      viewer.removeAllShapes();

      viewer.addPointCloud<PointType>(scene_pts, scene_color_handler, "scene pts"); 
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene pts");
      viewer.addPointCloud<PointType>(actual_model_pts, model_color_handler, "model pts gt"); 
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model pts gt");
 
      if (draw_mode == 0){
        // Draw ground truth correspondences
        for (int k_s=0; k_s<scene_pts->size(); k_s++){
          int k = correspondences_gt[k_s];
          std::stringstream ss_line;
          ss_line << "gt_correspondence_line" << k << "-" << k_s;
          viewer.addLine<PointType, PointType> (actual_model_pts->at(k), scene_pts->at(k_s), 0, 255, 0, ss_line.str ());
        }

        // Draw all correspondneces as lines
        int num=0;
        for (auto det = detections.begin(); det != detections.end(); det++){
          for (auto corr = det->correspondences.begin(); corr != det->correspondences.end(); corr++){
            std::stringstream ss_line;
            ss_line << "correspondence_line_" << num;
            num++;
            viewer.addLine<PointType, PointType> (corr->model_pt, corr->scene_pt, 255, 0, 255, ss_line.str());
          }
        }

      } else if (draw_mode == 1) {
        int num=0; int num2=0;
        for (auto det = detections.begin(); det != detections.end(); det++){
          auto corr = &(det->correspondences[target_corresp_id]);
          std::stringstream ss_line;
          ss_line << "correspondence_line_" << num;
          num++;
          viewer.addLine<PointType, PointType> (corr->model_pt, corr->scene_pt, 255, 0, 255, ss_line.str());
        }

        // And desired ground truth
        int k = correspondences_gt[target_corresp_id];
        int k_s = target_corresp_id;
        std::stringstream ss_line;
        ss_line << "gt_correspondence_line" << k << "-" << k_s;
        viewer.addLine<PointType, PointType> (actual_model_pts->at(k), scene_pts->at(k_s), 0, 255, 0, ss_line.str ());
      } else if (draw_mode == 2) { 
        // Draw only the estimated transformed model, which is handled below
      }


      // Always draw the transformed and untransformed models
      for (int i=0; i<models.size(); i++){
        auto verts = models[i].vertices;
        for (int j=0; j<models[i].faces.size(); j++){
          pcl::PointCloud<PointType>::Ptr face_pts (new pcl::PointCloud<PointType> ());
          for (int k=0; k<3; k++){
            face_pts->push_back(
              PointType( verts(0, models[i].faces[j][k]),
                         verts(1, models[i].faces[j][k]),
                         verts(2, models[i].faces[j][k]) ));
          } 
          char strname[100];

          /*
          if (draw_mode == 2){
            // model pts
            for (int k=0; k<detections.size(); k++){
              if (detections[k].obj_ind == i){
                sprintf(strname, "polygon%d_%d_%d", i, j, k);
                transformPointCloud(*face_pts, *face_pts, models[i].scene_transform.cast<float>());
                transformPointCloud(*face_pts, *face_pts, detections[k].est_tf.cast<float>());
                viewer.addPolygon<PointType>(face_pts, 0.5, 0.5, 1.0, string(strname));
                transformPointCloud(*face_pts, *face_pts, detections[k].est_tf.inverse().cast<float>());
                transformPointCloud(*face_pts, *face_pts, models[i].scene_transform.inverse().cast<float>());
              }            
            }
          } else {
            sprintf(strname, "polygon%d_%d", i, j);
            transformPointCloud(*face_pts, *face_pts, models[i].scene_transform.cast<float>());
            viewer.addPolygon<PointType>(face_pts, 0.5, 0.5, 1.0, string(strname));
            transformPointCloud(*face_pts, *face_pts, models[i].scene_transform.inverse().cast<float>());
          }
          */
          // scene pts
          sprintf(strname, "polygon%d_%d_tf", i, j);
          transformPointCloud(*face_pts, *face_pts, models[i].model_transform.cast<float>());
          viewer.addPolygon<PointType>(face_pts, 0.5, 1.0, 0.5, string(strname));
        }
      }

      // Always draw info
      std::stringstream ss_info;
      ss_info << "MILP Scene Point to Model Mesh Correspondences" << endl
              << "Solution Objective: " << objective << endl
              << "Drawing mode [Z]: " << draw_mode  << endl
              << "Drawing correspondence [Left/Right Keys]: " << target_corresp_id << endl
              << "Drawing solution [Up/Down Keys]: " << target_sol << endl;
      viewer.addText  ( ss_info.str(), 10, 10, "optim_info_str");
      

      pending_redraw = false;
    }

    viewer.spinOnce ();
  }

  return 0;
}
