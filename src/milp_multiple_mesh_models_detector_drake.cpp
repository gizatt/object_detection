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

const double kBigNumber = 100;

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
    struct PointCorrespondence {
      Vector3d scene_pt;
      Vector3d model_pt;
      std::vector<Vector3d> model_verts;
      std::vector<double> vert_weights;
    };

    struct ObjectDetection {
      Eigen::Affine3f est_tf;
      std::vector<PointCorrespondence> correspondences;
      int obj_ind;
      double objective;
    };

    MILPMultipleMeshModelDetector(YAML::Node config){
      if (config["rotation_constraint"])
        optRotationConstraint_ = config["rotation_constraint"].as<int>();
      if (config["rotation_constraint_num_faces"])
        optRotationConstraintNumFaces_ = config["rotation_constraint_num_faces"].as<int>();
      if (config["allow_outliers"])
        optAllowOutliers_ = config["allow_outliers"].as<bool>();
      if (config["phi_max"])
        optPhiMax_ = config["phi_max"].as<double>();
      if (config["use_initial_guess"])
        optUseInitialGuess_ = config["use_initial_guess"].as<bool>();
      if (config["rot_corruption"])
        optRotCorruption_ = config["rot_corruption"].as<double>();
      if (config["trans_corruption"])
        optTransCorruption_ = config["trans_corruption"].as<double>();

      config_ = config;

      // Load the model itself
      if (config["models"] == NULL){
        runtime_error("Must specify models for object detector to work with.");
      }
      // Model will be a RigidBodyTree.
      q_robot_gt_.resize(0);
      int old_q_robot_gt_size = 0;
      for (auto iter=config["models"].begin(); iter!=config["models"].end(); iter++){
        string urdf = (*iter)["urdf"].as<string>();
        AddModelInstanceFromUrdfFileWithRpyJointToWorld(urdf, &robot_);
        // And add initial state info that we were passed
        vector<double> q0 = (*iter)["q0"].as<vector<double>>();
        assert(robot_.get_num_positions() - old_q_robot_gt_size == q0.size());
        q_robot_gt_.conservativeResize(robot_.get_num_positions());
        for (int i=0; i<q0.size(); i++){
          q_robot_gt_[old_q_robot_gt_size] = q0[i];
          old_q_robot_gt_size++; 
        }
      }
    }

    std::vector<ObjectDetection> doObjectDetection(Eigen::Matrix3Xd scene_pts){
      KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_gt_);

      // Extract vertices and meshes from the RBT.
      Matrix3Xd all_vertices;
      DrakeShapes::TrianglesVector all_faces;
      vector<int> face_body_map;
      
      // Collect faces from each body
      int k=0;
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

              all_vertices.conservativeResize(3, points.cols() + all_vertices.cols());
              all_vertices.block(0, all_vertices.cols() - points.cols(), 3, points.cols()) = points;
              DrakeShapes::TrianglesVector faces;
              geometry.getFaces(faces);
              // Also use this loop to offset the vertex indices to the appropriate all_vertices indices.
              for (auto face : faces) {
                face[0] += (all_vertices.cols() - points.cols());
                face[1] += (all_vertices.cols() - points.cols());
                face[2] += (all_vertices.cols() - points.cols());
                face_body_map.push_back( k );
              }
              all_faces.insert(all_faces.end(), faces.begin(), faces.end());
              k++;
            } 
          }
        }
      }
      
      // And reform those extracted verticies and meshes into
      // matrices for our optimization
      MatrixXd F(all_faces.size(), all_vertices.cols());
      MatrixXd B(robot_.bodies.size(), all_faces.size());
      B.setZero();
      F.setZero();
      for (int i=0; i<all_faces.size(); i++){
        // Generate sub-block of face selection matrix F
        F(i, all_faces[i][0]) = 1.;
        F(i, all_faces[i][1]) = 1.;
        F(i, all_faces[i][2]) = 1.;
        B(face_body_map[i], i) = 1.0;
      }

      // See https://www.sharelatex.com/project/5850590c38884b7c6f6aedd1
      // for problem formulation
      MathematicalProgram prog;

      // Allocate slacks to choose minimum L-1 norm over objects
      auto phi = prog.NewContinuousVariables(scene_pts.cols(), 1, "phi");
      
      // And slacks to store term-wise absolute value terms for L-1 norm calculation
      auto alpha = prog.NewContinuousVariables(3, scene_pts.cols(), "alpha");

      // Each row is a set of affine coefficients relating the scene point to a combination
      // of vertices on a single face of the model
      auto C = prog.NewContinuousVariables(scene_pts.cols(), all_vertices.cols(), "C");
      // Binary variable selects which face is being corresponded to
      auto f = prog.NewBinaryVariables(scene_pts.cols(), F.rows(),"f");

      struct TransformationVars {
        VectorXDecisionVariable T;
        MatrixXDecisionVariable R;
      };
      std::vector<TransformationVars> transform_by_object;
      for (int i=0; i<robot_.bodies.size(); i++){
        TransformationVars new_tr;
        char name_postfix[100];
        sprintf(name_postfix, "_%s_%d", robot_.bodies[i]->get_model_name().c_str(), i);
        new_tr.T = prog.NewContinuousVariables(3, string("T")+string(name_postfix));
        prog.AddBoundingBoxConstraint(-100*VectorXd::Ones(3), 100*VectorXd::Ones(3), new_tr.T);
        new_tr.R = NewRotationMatrixVars(&prog, string("R") + string(name_postfix));

        if (optRotationConstraint_ > 0){
          switch (optRotationConstraint_){
            case 1:
              break;
            case 2:
              // Columnwise and row-wise L1-norm >=1 constraints
              for (int k=0; k<3; k++){
                prog.AddLinearConstraint(Vector3d::Ones().transpose(), 1.0, std::numeric_limits<double>::infinity(), new_tr.R.row(k).transpose());
                prog.AddLinearConstraint(Vector3d::Ones().transpose(), 1.0, std::numeric_limits<double>::infinity(), new_tr.R.col(k));
              }
              break;
            case 3:
              addMcCormickQuaternionConstraint(prog, new_tr.R, optRotationConstraintNumFaces_, optRotationConstraintNumFaces_);
              break;
            case 4:
              AddRotationMatrixMcCormickEnvelopeMilpConstraints(&prog, new_tr.R, optRotationConstraintNumFaces_);
              break;
            case 5:
              AddBoundingBoxConstraintsImpliedByRollPitchYawLimits(&prog, new_tr.R, kYaw_0_to_PI_2 | kPitch_0_to_PI_2 | kRoll_0_to_PI_2);
              break;
            default:
              printf("invalid optRotationConstraint_ option!\n");
              exit(-1);
              break;
          }
        } else {
          // constrain rotations to ground truth
          // I know I can do this in one constraint with 9 rows, but eigen was giving me trouble
          auto ground_truth_tf = robot_.relativeTransform(robot_kinematics_cache, 0, robot_.bodies[i]->get_body_index());
          for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
              prog.AddLinearEqualityConstraint(Eigen::MatrixXd::Identity(1, 1), ground_truth_tf.rotation()(i, j), new_tr.R.block<1,1>(i, j));
            }
          }
        }

        transform_by_object.push_back(new_tr);
      }

      // Optimization pushes on slacks to make them tight (make them do their job)
      prog.AddLinearCost(1.0 * VectorXd::Ones(scene_pts.cols()), phi);
      /*
      for (int k=0; k<3; k++){
        prog.AddLinearCost(1.0 * VectorXd::Ones(alpha.cols()), {alpha.row(k)});
      }./bias
      */

      // Constrain slacks nonnegative, to help the estimation of lower bound in relaxation  
      prog.AddBoundingBoxConstraint(0.0, std::numeric_limits<double>::infinity(), phi);
      for (int k=0; k<3; k++){
        prog.AddBoundingBoxConstraint(0.0, std::numeric_limits<double>::infinity(), {alpha.row(k).transpose()});
      }

      // Constrain each row of C to sum to 1 if a face is selected, to make them proper
      // affine coefficients
      Eigen::MatrixXd C1 = Eigen::MatrixXd::Ones(1, C.cols()+f.cols());
      // sum(C_i) = sum(f_i)
      // sum(C_i) - sum(f_i) = 0
      C1.block(0, C.cols(), 1, f.cols()) = -Eigen::MatrixXd::Ones(1, f.cols());
      for (size_t k=0; k<C.rows(); k++){
        prog.AddLinearEqualityConstraint(C1, 0, {C.row(k).transpose(), f.row(k).transpose()});
      }

      // Constrain each row of f to sum to 1, to force selection of exactly
      // one face to correspond to
      Eigen::MatrixXd f1 = Eigen::MatrixXd::Ones(1, f.cols());
      for (size_t k=0; k<f.rows(); k++){
        if (optAllowOutliers_){
          prog.AddLinearConstraint(f1, 0, 1, f.row(k).transpose()); 
        } else {
          prog.AddLinearEqualityConstraint(f1, 1, f.row(k).transpose());
        }
      }

      // Force all elems of C nonnegative
      for (int i=0; i<C.rows(); i++){
        for (int j=0; j<C.cols(); j++){
          prog.AddBoundingBoxConstraint(0.0, 1.0, C(i, j));
        }
      }

      // Force elems of C to be zero unless their corresponding vertex is a member
      // of an active face
      // That is,
      //   C_{i, j} <= F_{:, j}^T * f_{i, :}^T
      // or reorganized
      // [0] <= [F_{:, j}^T -1] [f_{i, :}^T C_{i, j}]
      //         
      for (int i=0; i<C.rows(); i++){
        for (int j=0; j<C.cols(); j++){
          MatrixXd A(1, F.rows() + 1);
          A.block(0, 0, 1, F.rows()) = F.col(j).transpose();
          A(0, F.rows()) = -1.0;

          prog.AddLinearConstraint(A, 0.0, std::numeric_limits<double>::infinity(), {f.row(i).transpose(), C.block<1,1>(i,j)});
        }
      }

      // I'm adding a dummy var constrained to zero to 
      // fill out the diagonals of C_i.
      auto C_dummy = prog.NewContinuousVariables(1, "c_dummy_zero");
      prog.AddLinearEqualityConstraint(Eigen::MatrixXd::Ones(1, 1), Eigen::MatrixXd::Zero(1, 1), C_dummy);

      // Helper variable to produce linear constraint
      // alpha_{i, l} +/- (R_l * s_i + T - M C_{i, :}^T) - Big * B_l * f_i >= -Big
      auto AlphaConstrPos = Eigen::RowVectorXd(1, 1+3+1+all_vertices.cols() + B.cols());    
      AlphaConstrPos.block<1, 1>(0, 0) = MatrixXd::Ones(1, 1); // multiplies alpha_{i, l} elem
      AlphaConstrPos.block<1, 1>(0, 4) = -1.0 * MatrixXd::Ones(1, 1); // T bias term
      auto AlphaConstrNeg = Eigen::RowVectorXd(1, 1+3+1+all_vertices.cols() + B.cols());    
      AlphaConstrNeg.block<1, 1>(0, 0) = MatrixXd::Ones(1, 1); // multiplies alpha_{i, l} elem
      AlphaConstrNeg.block<1, 1>(0, 4) = MatrixXd::Ones(1, 1); // T bias term

      printf("Starting to add correspondence costs... ");
      for (int l=0; l<robot_.bodies.size(); l++){
        AlphaConstrPos.block(0, 5+all_vertices.cols(), 1, B.cols()) = -kBigNumber*B.row(l); // multiplies f_i
        AlphaConstrNeg.block(0, 5+all_vertices.cols(), 1, B.cols()) = -kBigNumber*B.row(l); // multiplies f_i

        for (int i=0; i<scene_pts.cols(); i++){
          printf("=");

          // constrain L-1 distance slack based on correspondences
          // phi_i >= 1^T alpha_{i}
          // phi_i - 1&T alpha_{i} >= 0
          RowVectorXd PhiConstr(1 + 3);
          PhiConstr.setZero();
          PhiConstr(0, 0) = 1.0; // multiplies phi
          PhiConstr.block<1,3>(0,1) = -RowVector3d::Ones(); // multiplies alpha
          prog.AddLinearConstraint(PhiConstr, 0, std::numeric_limits<double>::infinity(),
          {phi.block<1,1>(i, 0),
           alpha.col(i)});

          // If we're allowing outliers, we need to constrain each phi_i to be bigger than
          // a penalty amount if no faces are selected
          if (optAllowOutliers_){
            // phi_i >= phi_max - (ones * f_i)*BIG
            // -> phi_ + (ones * f_i)*BIG >= phi_max
            Eigen::MatrixXd A_phimax = Eigen::MatrixXd::Ones(1, f.cols() + 1);
            A_phimax.block(0, 0, 1, f.cols()) = Eigen::MatrixXd::Ones(1, f.cols())*kBigNumber;
            for (size_t k=0; k<f.rows(); k++){
              prog.AddLinearConstraint(A_phimax, optPhiMax_, std::numeric_limits<double>::infinity(), {f.row(k).transpose(), phi.row(k)});
            }
          }

          // Alphaconstr, containing the scene and model points and a translation bias term, is used the constraints
          // on the three elems of alpha_{i, l}
          auto s_xyz = scene_pts.col(i);
          AlphaConstrPos.block<1, 3>(0, 1) = -s_xyz.transpose(); // Multiples R
          AlphaConstrNeg.block<1, 3>(0, 1) = s_xyz.transpose(); // Multiples R

          AlphaConstrPos.block(0, 5, 1, all_vertices.cols()) = 1.0 * all_vertices.row(0); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrPos, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(0, i),
             transform_by_object[l].R.block<1, 3>(0, 0).transpose(), 
             transform_by_object[l].T.block<1,1>(0,0),
             C.row(i).transpose(),
             f.row(i).transpose()});

          AlphaConstrPos.block(0, 5, 1, all_vertices.cols()) = 1.0 * all_vertices.row(1); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrPos, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(1, i),
             transform_by_object[l].R.block<1, 3>(1, 0).transpose(), 
             transform_by_object[l].T.block<1,1>(1,0),
             C.row(i).transpose(),
             f.row(i).transpose()});

          AlphaConstrPos.block(0, 5, 1, all_vertices.cols()) = 1.0 * all_vertices.row(2); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrPos, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(2, i),
             transform_by_object[l].R.block<1, 3>(2, 0).transpose(), 
             transform_by_object[l].T.block<1,1>(2,0),
             C.row(i).transpose(),
             f.row(i).transpose()});

          AlphaConstrNeg.block(0, 5, 1, all_vertices.cols()) = -1.0 * all_vertices.row(0); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrNeg, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(0, i),
             transform_by_object[l].R.block<1, 3>(0, 0).transpose(), 
             transform_by_object[l].T.block<1,1>(0,0),
             C.row(i).transpose(),
             f.row(i).transpose()});
          AlphaConstrNeg.block(0, 5, 1, all_vertices.cols()) = -1.0 * all_vertices.row(1); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrNeg, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(1, i),
             transform_by_object[l].R.block<1, 3>(1, 0).transpose(), 
             transform_by_object[l].T.block<1,1>(1,0),
             C.row(i).transpose(),
             f.row(i).transpose()});
          AlphaConstrNeg.block(0, 5, 1, all_vertices.cols()) = -1.0 * all_vertices.row(2); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrNeg, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(2, i),
             transform_by_object[l].R.block<1, 3>(2, 0).transpose(), 
             transform_by_object[l].T.block<1,1>(2,0),
             C.row(i).transpose(),
             f.row(i).transpose()});
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

      if (config_["gurobi_int_options"]){
        for (auto iter = config_["gurobi_int_options"].begin();
             iter != config_["gurobi_int_options"].end();
             iter++){
          prog.SetSolverOption(SolverType::kGurobi, iter->first.as<string>(), iter->second.as<int>());
        }
      }
      if (config_["gurobi_float_options"]){
        for (auto iter = config_["gurobi_float_options"].begin();
             iter != config_["gurobi_float_options"].end();
             iter++){
          prog.SetSolverOption(SolverType::kGurobi, iter->first.as<string>(), iter->second.as<float>());
        }
      }

      if (optUseInitialGuess_){
        // Corruption should be done from q_gt, then recover maximal coordinates from transform queries,
        // get mesh and do projection using get_closest_points in drake 
        /*

        // corrupt scene tfs by a bit
        for (int model_i=0; model_i<robot_.bodies.size(); model_i++){
          Affine3d scene_tf_corruption;
          scene_tf_corruption.setIdentity();
          scene_tf_corruption.translation() = Vector3d(randrange(-optTransCorruption_, optTransCorruption_),
                                                       randrange(-optTransCorruption_, optTransCorruption_),
                                                       randrange(-optTransCorruption_, optTransCorruption_));
          scene_tf_corruption.rotate(AngleAxisd(randrange(-optRotCorruption_, optRotCorruption_), Vector3d::UnitZ()));
          scene_tf_corruption.rotate(AngleAxisd(randrange(-optRotCorruption_, optRotCorruption_), Vector3d::UnitY()));
          scene_tf_corruption.rotate(AngleAxisd(randrange(-optRotCorruption_, optRotCorruption_), Vector3d::UnitX()));
          scene_tf_corruption  = scene_tf_corruption * 

          prog.SetInitialGuess(transform_by_object[model_i].T, scene_tf_corruption.translation());
          prog.SetInitialGuess(transform_by_object[model_i].R, scene_tf_corruption.matrix().block<3, 3>(0, 0));
        }

        // for every scene point, project it down onto the models at the supplied TF to get closest face, and use 
        // that face assignment as our guess
        printf("Doing a bunch of inefficient point projections...\n");

        // Each row is a set of affine coefficients relating the scene point to a combination
        // of vertices on a single face of the model
        //MatrixXd C0(scene_pts.cols(), vertices.cols());
        //C0.setZero();
        // Binary variable selects which face is being corresponded to
        MatrixXd f0(scene_pts.cols(), F.rows());
        f0.setZero();
        for (int i=0; i<scene_pts.cols(); i++){
          printf("\rGenerating guess for point (%d)/(%d)", i, (int)scene_pts.cols());
          double dist = std::numeric_limits<double>::infinity();
          Vector3d closest_pt;
          int model_ind;
          int face_ind;
          projectOntoModelSet(pointToVector3d(scene_pts->at(i)), corrupted_models, &dist, &closest_pt, &model_ind, &face_ind);
          if (dist < 0.1){
            f0(i, face_ind) = 1;
          }
          // else it's an outlier point
        }
        prog.SetInitialGuess(f, f0);
        */
      }

      //  prog.SetSolverOption(SolverType::kGurobi, "Cutoff", 50.0);
      // isn't doing anything... not invoking this tool right?
      //  prog.SetSolverOption(SolverType::kGurobi, "TuneJobs", 8);
      //  prog.SetSolverOption(SolverType::kGurobi, "TuneResults", 3);
      //prog.SetSolverOption(SolverType::kGurobi, )

      auto out = gurobi_solver.Solve(prog);
      string problem_string = "rigidtf";
      double elapsed = getUnixTime() - now;

      //prog.PrintSolution();
      printf("Code %d, problem %s solved for %lu scene solved in: %f\n", out, problem_string.c_str(), scene_pts.cols(), elapsed);


      std::vector<ObjectDetection> detections(prog.get_num_solutions());
      return detections;
    }

  private:
    RigidBodyTree<double> robot_;
    VectorXd q_robot_gt_;

    YAML::Node config_;

    int optNumRays_ = 10;
    int optRotationConstraint_ = 4;
    int optRotationConstraintNumFaces_ = 2;
    bool optAllowOutliers_ = true;
    double optPhiMax_ = 0.1;
    bool optUseInitialGuess_ = false;
    double optRotCorruption_ = 0.1;
    double optTransCorruption_ = 0.1;

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
  pcl::PointCloud<PointType>::Ptr scene_pts_pcl = pcg.samplePointCloudFromSurface();
  // Convert it to Eigen
  Matrix3Xd scene_pts(3, scene_pts_pcl->size());
  for (int i=0; i<scene_pts_pcl->size(); i++){
    scene_pts.col(i) = pointToVector3d(scene_pts_pcl->at(i));
  }

  // And set up our detector
  if (config["detector_options"] == NULL){
    runtime_error("Config needs a detector option set.");
  }
  MILPMultipleMeshModelDetector detector(config["detector_options"]);
  auto detections = detector.doObjectDetection(scene_pts);

  // Visualize the generated model point cloud
  pcl::visualization::PCLVisualizer viewer ("Point Collection");

  pcl::visualization::PointCloudColorHandlerCustom<PointType> model_color_handler (scene_pts_pcl, 128, 255, 255);
  viewer.addPointCloud<PointType>(scene_pts_pcl, model_color_handler, "model pts"); 
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model pts");

  while (!viewer.wasStopped ())
    viewer.spinOnce ();

  return 0;
}
