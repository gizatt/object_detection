#pragma once

#include <stdexcept>
#include <iostream>

#include "drake/solvers/mathematical_program.h"
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

using namespace std;
using namespace Eigen;
using namespace drake::solvers;

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;

Vector3d pointToVector3d(PointType & point){
  return Vector3d(point.x, point.y, point.z);
}
PointType Vector3dToPoint(Vector3d & vec){
  return PointType(vec(0), vec(1), vec(2));
}