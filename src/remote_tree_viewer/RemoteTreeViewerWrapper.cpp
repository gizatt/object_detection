#include "RemoteTreeViewerWrapper.hpp"

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include "nlohmann/json.hpp"

#include "../common/common.hpp"

#include "lcmtypes/robotlocomotion/viewer2_comms_t.hpp"

using namespace std;
using namespace Eigen;
using namespace drake::lcm;
using namespace DrakeShapes;
using json = nlohmann::json;
using namespace robotlocomotion;

RemoteTreeViewerWrapper::RemoteTreeViewerWrapper() {
}

void RemoteTreeViewerWrapper::publishPointCloud(const Matrix3Xd pts, vector<string> path){
  long long int now = getUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {
    {"timestamp", now},
    {"setgeometry", 
      {{
        {"path", path},
        {"geometry",
          {
            {"type", "pointcloud"},
            {"points", vector<vector<double>>()},
            {"channels",
              {
                {"rgb", vector<vector<double>>()}
              }
            }
          }
        }
      }},
    },
    {"settransform", json({})},
    {"delete", json({})}
  };

  // Push in the points and colors.
  for (int i=0; i<pts.cols(); i++){
    j["setgeometry"][0]["geometry"]["points"].push_back( {pts(0, i), pts(1, i), pts(2, i)} );
    j["setgeometry"][0]["geometry"]["channels"]["rgb"].push_back( {1.0, 0.0, 1.0} );
  }

  auto msg = viewer2_comms_t();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : j.dump())
    msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::publishRigidBodyTree(const RigidBodyTree<double>& tree, const VectorXd q, vector<string> path){
  auto kinematics_cache = tree.doKinematics(q);
  for (const auto& body : tree.bodies) {
    for (const auto& collision_elem_id : body->get_collision_element_ids()) {
      auto element = tree.FindCollisionElement(collision_elem_id);
      if (element->hasGeometry()){
        vector<string> full_name = path;
        full_name.push_back(body->get_name());
        publishGeometry(element->getGeometry(),
          tree.relativeTransform(kinematics_cache, 0, body->get_body_index()), 
          full_name);
      }
    }
  }
}

void RemoteTreeViewerWrapper::publishRigidBody(const RigidBody<double>& body, Affine3d tf, vector<string> path){
}

void RemoteTreeViewerWrapper::publishGeometry(const Geometry& geometry, Affine3d tf, vector<string> path){
  long long int now = getUnixTime() * 1000 * 1000;

  // Short-circuit to points if the passed geometry is a set of mesh points
  if (geometry.getShape() == MESH_POINTS){
    Matrix3Xd pts;
    geometry.getPoints(pts);
    publishPointCloud(pts, path);
  }

  // Extract std::vector-formatted translation and quaternion from the tf
  vector<double> translation = {
    tf.translation()[0], 
    tf.translation()[1], 
    tf.translation()[2]};
  Quaterniond q(tf.rotation());
  vector<double> rotation = { q.w(), q.x(), q.y(), q.z() };

  // Format a JSON string for this bit of geometry (in a setgeometry call)
  // and its transformation (in a settransform call)
  json j = {
    {"timestamp", now},
    {"setgeometry", 
      {{
        {"path", path},
        {"geometry", json({})}
      }},
    },
    {"settransform", 
      {{
        {"path", path},
        {"transform", 
          {
            {"translation", translation},
            {"quaternion", rotation}
          }
        }
      }}
    },
    {"delete", json({})}
  };

  // Fill in the setgeometry call based on what kind of geometry this is
  switch (geometry.getShape()){
    case BOX:
      {
      const Box * box = static_cast<const Box *>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = string("box");
      j["setgeometry"][0]["geometry"]["lengths"] = vector<double>({box->size[0], box->size[1], box->size[2]});
      }
      break;
    case SPHERE:
      {
      const Sphere * sphere = static_cast<const Sphere *>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = string("sphere");
      j["setgeometry"][0]["geometry"]["radius"] = sphere->radius;
      }
      break;
    case CYLINDER:
      {
      const Cylinder * cylinder = static_cast<const Cylinder *>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = string("cylinder");
      j["setgeometry"][0]["geometry"]["radius"] = cylinder->radius;
      j["setgeometry"][0]["geometry"]["length"] = cylinder->length;
      }
      break;
    case MESH:
      {
      const Mesh * mesh = static_cast<const Mesh *>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = string("mesh_file");
      j["setgeometry"][0]["geometry"]["filename"] = mesh->resolved_filename_;
      }
      break;
    case CAPSULE:      {
      const Capsule * capsule = static_cast<const Capsule *>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = string("capsule");
      j["setgeometry"][0]["geometry"]["radius"] = capsule->radius;
      j["setgeometry"][0]["geometry"]["length"] = capsule->length;
      }
      break;
    default:
      cout << "Unsupported geometry type " << geometry.getShape() << ", sorry!" << cout;
      return;
  }


  auto msg = viewer2_comms_t();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : j.dump())
    msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}