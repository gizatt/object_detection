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

using namespace std;
using namespace Eigen;
using namespace drake::solvers;
using namespace drake::parsers::urdf;

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;

const double kBigNumber = 100.0; // must be bigger than largest possible correspondance distance

MatrixXd boundingBox2FaceSel(Matrix3Xd bb_pts){

  // Get average per axis, which we'll use to distinguish
  // the postive vs negative face on each axis
  Vector3d avg;
  avg.setZero();
  for (int k=0; k<bb_pts.cols(); k++){
    avg += bb_pts.col(k);
  }
  avg /= (double)bb_pts.cols();

  // order in:
  // cx << -1, 1, 1, -1, -1, 1, 1, -1;
  // cy << 1, 1, 1, 1, -1, -1, -1, -1;
  // cz << 1, 1, -1, -1, -1, -1, 1, 1;
  MatrixXd F(6, bb_pts.cols());
  F.setZero();

  vector<vector<Vector3d>> out;
  int k=0;
  for (int xyz=0; xyz<3; xyz+=1){
    for (int tar=-1; tar<=1; tar+=2){
      for (int i=0; i<8; i++){
        if ((bb_pts(xyz, i)-avg(xyz))*(double)tar >= 0){
          F(k, i) = 1.0;
        }
      }
      k++;
    }
  }
  return F;
}

Vector3d pointToVector3d(PointType & point){
  return Vector3d(point.x, point.y, point.z);
}
PointType Vector3dToPoint(Vector3d & vec){
  return PointType(vec(0), vec(1), vec(2));
}

vector< vector<Vector3d> > boundingBox2QuadMesh(MatrixXd bb_pts){
  // order in:
  // cx << -1, 1, 1, -1, -1, 1, 1, -1;
  // cy << 1, 1, 1, 1, -1, -1, -1, -1;
  // cz << 1, 1, -1, -1, -1, -1, 1, 1;
  vector<vector<Vector3d>> out;
  for (int xyz=0; xyz<3; xyz+=1){
    for (int tar=-1; tar<=1; tar+=2){
      vector<Vector3d> face;
      for (int i=0; i<8; i++){
        if (bb_pts(xyz, i)*(double)tar >= 0){
          face.push_back(bb_pts.col(i));
        }
      }
      out.push_back(face);
    }
  }
  return out;
}

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

  if (argc != 2){
    printf("Use: miqp_4pcs_point_model_detector <config file>\n");
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

  // Generate the point quads we'll use
  //   a *
  //      --      ---* d
  //        -- ---
  //        --*e
  //  c *---    --
  //              --
  //                * b
  struct PointQuad {
    int ind_a;
    int ind_b;
    int ind_c;
    int ind_d;
    Vector3d a;
    Vector3d b;
    Vector3d c;
    Vector3d d;
    Vector3d e;
    double r_1;
    double r_2;
  };
  printf("Generating point quads");
  vector<PointQuad> point_quads(optNumPointQuads);
  int i=0;
  while (i < optNumPointQuads){
    int ind_a = rand() % scene_pts->size();
    int ind_b = rand() % scene_pts->size();
    int ind_c = rand() % scene_pts->size();
    if (ind_a != ind_b && ind_b != ind_c && ind_a != ind_c){
      point_quads[i].a = pointToVector3d(scene_pts->at(ind_a));
      point_quads[i].b = pointToVector3d(scene_pts->at(ind_b));
      point_quads[i].c = pointToVector3d(scene_pts->at(ind_c));
      point_quads[i].ind_a = ind_a;
      point_quads[i].ind_b = ind_b;
      point_quads[i].ind_c = ind_c;

      // find vert that is the least out-of-plane
      // TODO(gizatt) maybe I should pick one that is slightly out of plane instead?
      Vector3d norm_vector = (point_quads[i].a - point_quads[i].b).cross(point_quads[i].a - point_quads[i].c);
      norm_vector /= norm_vector.norm();

      double best_oop_dist = 10000000.0;
      bool found_one = false;
      for (int j=0; j<scene_pts->size(); j++){
        double oop_dist = fabs((pointToVector3d(scene_pts->at(j)) - point_quads[i].a).transpose() * norm_vector);
        if (j != ind_a && j != ind_b && j != ind_c && 
          oop_dist < best_oop_dist){
          point_quads[i].d = pointToVector3d(scene_pts->at(j));
          point_quads[i].ind_d = j;
          best_oop_dist = oop_dist;
          found_one = true;
        }
      }

      if (found_one){
        // We now have four approximately-coplanar points
        // we need to find e, which may require flipping c and d to get the lines to cross
        // Take the flipping of b and c that results in the intersection that results in an r
        // between 0 and 1 (i.e. intersection between a/b and c/d
        MatrixXd A(3, 2);
        Vector3d b;

        Vector3d along_ab = point_quads[i].b - point_quads[i].a; along_ab /= along_ab.norm();
        Vector3d along_cd = point_quads[i].d - point_quads[i].c; along_cd /= along_cd.norm();
        A << along_ab, -along_cd;
        b = point_quads[i].c - point_quads[i].a;
        Vector2d out = A.colPivHouseholderQr().solve(b);
        Vector3d e_ab = out(0)*along_ab + point_quads[i].a;
        Vector3d e_cd = out(1)*along_cd + point_quads[i].c;
        double r_1_abcd = out(0) / (point_quads[i].b - point_quads[i].a).norm();
        double r_2_abcd = out(1) / (point_quads[i].d - point_quads[i].c).norm();
        double dist_abcd = (e_ab - e_cd).norm();
        if (r_1_abcd < 0.0 or r_1_abcd > 1.0 or r_2_abcd < 0.0 or r_2_abcd > 1.0){
          dist_abcd = 99999999;
        }


        Vector3d along_ac = point_quads[i].c - point_quads[i].a; along_ac /= along_ac.norm();
        Vector3d along_bd = point_quads[i].d - point_quads[i].b; along_bd /= along_bd.norm();
        A << along_ac, -along_bd;
        b = point_quads[i].b - point_quads[i].a;
        out = A.colPivHouseholderQr().solve(b);
        Vector3d e_ac = out(0)*along_ac + point_quads[i].a;
        Vector3d e_bd = out(1)*along_bd + point_quads[i].b;
        double r_1_acbd = out(0) / (point_quads[i].c - point_quads[i].a).norm();
        double r_2_acbd = out(1) / (point_quads[i].d - point_quads[i].b).norm();
        double dist_acbd = (e_ac - e_bd).norm();
        if (r_1_acbd < 0.0 or r_1_acbd > 1.0 or r_2_acbd < 0.0 or r_2_acbd > 1.0){
          dist_acbd = 99999999;
        }

        Vector3d along_ad = point_quads[i].d - point_quads[i].a; along_ad /= along_ad.norm();
        Vector3d along_bc = point_quads[i].c - point_quads[i].b; along_bc /= along_bc.norm();
        A << along_ad, -along_bc;
        b = point_quads[i].b - point_quads[i].a;
        out = A.colPivHouseholderQr().solve(b);
        Vector3d e_ad = out(0)*along_ad + point_quads[i].a;
        Vector3d e_bc = out(1)*along_bc + point_quads[i].b;
        double r_1_adbc = out(0) / (point_quads[i].d - point_quads[i].a).norm();
        double r_2_adbc = out(1) / (point_quads[i].c - point_quads[i].b).norm();
        double dist_adbc = (e_ad - e_bc).norm();
        if (r_1_adbc < 0.0 or r_1_adbc > 1.0 or r_2_adbc < 0.0 or r_2_adbc > 1.0){
          dist_adbc = 99999999;
        }

        printf("(%f, %f: %f) vs (%f, %f: %f) vs (%f, %f: %f)\n", r_1_abcd, r_2_abcd, dist_abcd, r_1_acbd, r_2_acbd, dist_acbd, r_1_adbc, r_2_adbc, dist_adbc);
        // Prefer intersection that puts intersection between the four points, but this isn't
        // always possible
        if (dist_abcd < 10000 and dist_abcd < dist_acbd and dist_abcd < dist_adbc){
          printf("going with abcd\n");
          point_quads[i].e = e_ab;
          point_quads[i].r_1 = r_1_abcd;
          point_quads[i].r_2 = r_2_abcd;
          i++;
        } else if (dist_acbd < 10000 and dist_acbd < dist_abcd and dist_acbd < dist_adbc){
          printf("going with acbd\n");
          point_quads[i].e = e_ac;
          Vector3d buf = point_quads[i].b;
          int old_b_ind = point_quads[i].ind_b;
          point_quads[i].b = point_quads[i].c;
          point_quads[i].ind_b = point_quads[i].ind_c;
          point_quads[i].c = buf;
          point_quads[i].ind_c = old_b_ind;
          point_quads[i].r_1 = r_1_acbd;
          point_quads[i].r_2 = r_2_acbd;
          i++;
        } else if (dist_adbc < 10000 and dist_adbc < dist_abcd and dist_adbc < dist_acbd){
          printf("going with adbc\n");
          point_quads[i].e = e_ad;
          Vector3d buf = point_quads[i].b;
          int old_b_ind = point_quads[i].ind_b;
          point_quads[i].b = point_quads[i].d;
          point_quads[i].ind_b = point_quads[i].ind_d;
          point_quads[i].d = buf;
          point_quads[i].ind_d = old_b_ind;
          point_quads[i].r_1 = r_1_adbc;
          point_quads[i].r_2 = 1.0 - r_2_adbc; // was calculated in b->c direction, but the flip we just did results in c->d direction
          i++;
        } 
      }
    }
    printf("%d,", i);
  }
  printf("\n");


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

  // I'm adding a dummy var constrained to zero to 
  // fill out the diagonals of C_i.
  auto C_dummy = prog.NewContinuousVariables(1, "c_dummy_zero");
  prog.AddLinearEqualityConstraint(Eigen::MatrixXd::Ones(1, 1), Eigen::MatrixXd::Zero(1, 1), C_dummy);

  vector<bool> used_in_a_quad(scene_pts->size(), false);
  printf("Starting to add correspondence costs... ");
  for (int i=0; i<optNumPointQuads; i++){
    printf("=");

    // add lower-bound L1-norm distance constraints on each point pair in the quad
    // to prevent degenerate sols where a = b and/or c = d
    // constrain ||x_1 - x_2||_1 >= 
    Vector3d * pts[4] = {&point_quads[i].a,
                         &point_quads[i].b,
                         &point_quads[i].c,
                         &point_quads[i].d};
    int inds[4] = {point_quads[i].ind_a,
                     point_quads[i].ind_b,
                     point_quads[i].ind_c,
                     point_quads[i].ind_d};
    char buf[100];

    for (int k=0; k<actual_model_pts->size(); k++){
      prog.AddLinearConstraint(MatrixXd::Ones(1, 4), 0.0, 1.0, {C.block<1, 1>(inds[0], k),
                                                                C.block<1, 1>(inds[1], k),
                                                                C.block<1, 1>(inds[2], k),
                                                                C.block<1, 1>(inds[3], k)}); 
    }

    for (int u=0; u<4; u++){
      // Constrain that we MUST match this scene point with something
      prog.AddLinearEqualityConstraint(MatrixXd::Ones(1, actual_model_pts->size()), 1.0, C.row(inds[u]));
      used_in_a_quad[inds[u]] = true;
    }

    double r_1 = point_quads[i].r_1;
    double r_2 = point_quads[i].r_2;
    printf("%f, %f\n", r_1, r_2);

    // want to constraint
    // min || ( (1 - r_1) * C_a a + (r_1) C_b b) - ( (1 - r_2) * C_d c + (r_2) C_d d ) ||_2^2
    // which we'll separate out to x, y, and z components to make writing it out easier

    // simplifying to || B x ||_2^2, x = [C_a; C_b; C_c; C_d]
    //                               B = [(1-r_1) V_x, r_1 V_x, -(1-r_2) V_x, -r_2 V_x]
    int total_model_pts = actual_model_pts->size();
    MatrixXd B_x(1, total_model_pts + total_model_pts + total_model_pts + total_model_pts);
    MatrixXd B_y(1, total_model_pts + total_model_pts + total_model_pts + total_model_pts);
    MatrixXd B_z(1, total_model_pts + total_model_pts + total_model_pts + total_model_pts);
    double coeffs[4] = {1.-r_1, r_1, -(1.-r_2), -r_2};
    for (int k=0; k<4; k++){
      B_x.block(0, k*total_model_pts, 1, total_model_pts) = coeffs[k]*model_pts.row(0);
      B_y.block(0, k*total_model_pts, 1, total_model_pts) = coeffs[k]*model_pts.row(1);
      B_z.block(0, k*total_model_pts, 1, total_model_pts) = coeffs[k]*model_pts.row(2);
    }

    prog.AddQuadraticCost( B_x.transpose() * B_x, MatrixXd::Zero(1, 1), {C.row(point_quads[i].ind_a), 
                                                                         C.row(point_quads[i].ind_b), 
                                                                         C.row(point_quads[i].ind_c), 
                                                                         C.row(point_quads[i].ind_d)});
    prog.AddQuadraticCost( B_y.transpose() * B_y, MatrixXd::Zero(1, 1), {C.row(point_quads[i].ind_a), 
                                                                         C.row(point_quads[i].ind_b), 
                                                                         C.row(point_quads[i].ind_c), 
                                                                         C.row(point_quads[i].ind_d)});
    prog.AddQuadraticCost( B_z.transpose() * B_z, MatrixXd::Zero(1, 1), {C.row(point_quads[i].ind_a), 
                                                                         C.row(point_quads[i].ind_b), 
                                                                         C.row(point_quads[i].ind_c), 
                                                                         C.row(point_quads[i].ind_d)});
  }
  printf("\n");

  for (int i=0; i<scene_pts->size(); i++){
    if (!used_in_a_quad[i]){
      // force no assignment for this point
      prog.AddLinearEqualityConstraint(MatrixXd::Ones(1, actual_model_pts->size()), 0.0, C.row(i));
    }
  }
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
  struct PointQuadCorrespondence {
    PointQuad scene_quad;
    std::vector<int> model_pt_ind; 
    std::vector<PointType> model_pt; 
  };

  struct ObjectDetection {
    Eigen::Affine3f est_tf;
    std::vector<PointQuadCorrespondence> correspondences;
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

  max_corresp_id = optNumPointQuads;
  pending_redraw = true;

  std::vector<ObjectDetection> detections;
  MatrixXd f_est;
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

/*
        Vector3f Tf = prog.GetSolution(transform_by_object[i].T, target_sol).cast<float>();
        Matrix3f Rf = prog.GetSolution(transform_by_object[i].R, target_sol).cast<float>();
        printf("Transform:\n");
        printf("\tTranslation: %f, %f, %f\n", Tf(0, 0), Tf(1, 0), Tf(2, 0));
        printf("\tRotation:\n");
        printf("\t\t%f, %f, %f\n", Rf(0, 0), Rf(0, 1), Rf(0, 2));
        printf("\t\t%f, %f, %f\n", Rf(1, 0), Rf(1, 1), Rf(1, 2));
        printf("\t\t%f, %f, %f\n", Rf(2, 0), Rf(2, 1), Rf(2, 2));
        printf("------------------------------------------------\n");
        printf("Sanity check rotation: R^T R = \n");
        MatrixXf RfTRf = Rf.transpose() * Rf;
        printf("\t\t%f, %f, %f\n", RfTRf(0, 0), RfTRf(0, 1), RfTRf(0, 2));
        printf("\t\t%f, %f, %f\n", RfTRf(1, 0), RfTRf(1, 1), RfTRf(1, 2));
        printf("\t\t%f, %f, %f\n", RfTRf(2, 0), RfTRf(2, 1), RfTRf(2, 2));
        printf("------------------------------------------------\n");
        printf("************************************************\n");
*/

        detection.est_tf.setIdentity();
//        detection.est_tf.translation() = Tf;
//        detection.est_tf.matrix().block<3,3>(0,0) = Rf;

        for (int quad_i=0; quad_i<optNumPointQuads; quad_i++){
          PointQuadCorrespondence new_corresp;
          new_corresp.scene_quad = point_quads[quad_i];
          int inds[4] = {new_corresp.scene_quad.ind_a,
                         new_corresp.scene_quad.ind_b,
                         new_corresp.scene_quad.ind_c,
                         new_corresp.scene_quad.ind_d};
          new_corresp.model_pt.resize(4);
          new_corresp.model_pt_ind.resize(4);
          // for each of the four member correspondences, go find what face and vert was corresponded to
          for (int k=0; k<4; k++){
            for (int model_pt_j=0; model_pt_j<model_pts.cols(); model_pt_j++){
              if (C_est(inds[k], model_pt_j) > 0.5){
                new_corresp.model_pt[k] = actual_model_pts->at(model_pt_j);
                new_corresp.model_pt_ind[k] = model_pt_j;
                continue;
              }
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
        for (int k=0; k<point_quads.size(); k++){
          int p = correspondences_gt[point_quads[k].ind_a];
          std::stringstream ss_line;
          ss_line << "gt_correspondence_line" << k << "-a-" << p;
          viewer.addLine<PointType, PointType> (actual_model_pts->at(point_quads[k].ind_a), scene_pts->at(p), 0, 255, 0, ss_line.str ());   

          p = correspondences_gt[point_quads[k].ind_b];
          ss_line.clear();
          ss_line << "gt_correspondence_line" << k << "-b-" << p;
          viewer.addLine<PointType, PointType> (actual_model_pts->at(point_quads[k].ind_b), scene_pts->at(p), 0, 255, 0, ss_line.str ());   

          p = correspondences_gt[point_quads[k].ind_c];
          ss_line.clear();
          ss_line << "gt_correspondence_line" << k << "-c-" << p;
          viewer.addLine<PointType, PointType> (actual_model_pts->at(point_quads[k].ind_c), scene_pts->at(p), 0, 255, 0, ss_line.str ());   

          p = correspondences_gt[point_quads[k].ind_d];
          ss_line.clear();
          ss_line << "gt_correspondence_line" << k << "-d-" << p;
          viewer.addLine<PointType, PointType> (actual_model_pts->at(point_quads[k].ind_d), scene_pts->at(p), 0, 255, 0, ss_line.str ());   
        }

        // Draw all correspondneces as lines
        int num=0;
        for (auto det = detections.begin(); det != detections.end(); det++){
          for (auto corr = det->correspondences.begin(); corr != det->correspondences.end(); corr++){
            int inds[4] = {corr->scene_quad.ind_a, corr->scene_quad.ind_b, corr->scene_quad.ind_c, corr->scene_quad.ind_d};
            for (int k=0; k<4; k++){
              std::stringstream ss_line;
              ss_line << "correspondence_line_" << num;
              num++;
              viewer.addLine<PointType, PointType> (corr->model_pt[k], scene_pts->at(inds[k]), 255, 0, 255, ss_line.str());
            }
          }
        }

      } else if (draw_mode == 1) {
        int num=0; int num2=0;
        for (auto det = detections.begin(); det != detections.end(); det++){
          auto corr = &(det->correspondences[target_corresp_id]);
          int inds[4] = {corr->scene_quad.ind_a, corr->scene_quad.ind_b, corr->scene_quad.ind_c, corr->scene_quad.ind_d};
          for (int k=0; k<4; k++){
            std::stringstream ss_line;
            ss_line << "correspondence_line_" << num;
            num++;
            viewer.addLine<PointType, PointType> (corr->model_pt[k], scene_pts->at(inds[k]), 255, 0, 255, ss_line.str());
          }
          viewer.addSphere<PointType>( Vector3dToPoint(corr->scene_quad.e), 0.0025, 0., 0., 255., "e");
          viewer.addLine<PointType, PointType> ( Vector3dToPoint(corr->scene_quad.a), Vector3dToPoint(corr->scene_quad.b), 0, 125, 125, "ab");
          viewer.addLine<PointType, PointType> ( Vector3dToPoint(corr->scene_quad.c), Vector3dToPoint(corr->scene_quad.d), 125, 125, 0, "cd");
        }

        // And desired ground truth
        int k = target_corresp_id;
        int p = correspondences_gt[point_quads[k].ind_a];
        std::stringstream ss_line;
        ss_line << "gt_correspondence_line" << k << "-a-" << p;
        viewer.addLine<PointType, PointType> (actual_model_pts->at(point_quads[k].ind_a), scene_pts->at(p), 0, 255, 0, ss_line.str ());   

        p = correspondences_gt[point_quads[k].ind_b];
        ss_line.clear();
        ss_line << "gt_correspondence_line" << k << "-b-" << p;
        viewer.addLine<PointType, PointType> (actual_model_pts->at(point_quads[k].ind_b), scene_pts->at(p), 0, 255, 0, ss_line.str ());   

        p = correspondences_gt[point_quads[k].ind_c];
        ss_line.clear();
        ss_line << "gt_correspondence_line" << k << "-c-" << p;
        viewer.addLine<PointType, PointType> (actual_model_pts->at(point_quads[k].ind_c), scene_pts->at(p), 0, 255, 0, ss_line.str ());   

        p = correspondences_gt[point_quads[k].ind_d];
        ss_line.clear();
        ss_line << "gt_correspondence_line" << k << "-d-" << p;
        viewer.addLine<PointType, PointType> (actual_model_pts->at(point_quads[k].ind_d), scene_pts->at(p), 0, 255, 0, ss_line.str ());  
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
