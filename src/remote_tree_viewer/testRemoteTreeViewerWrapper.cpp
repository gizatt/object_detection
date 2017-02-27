#include "RemoteTreeViewerWrapper.hpp"
#include "../common/common.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"

using namespace Eigen;
using namespace drake::parsers::urdf;

int main(int argc, char ** argv){
	RemoteTreeViewerWrapper rm;

	int n_points = 1000;
	double x_size = 3.0;
	double y_size = 2.0;
	double z_size = 1.0;
	Matrix3Xd pts(3, n_points);
	for (int i=0; i<n_points; i++){
		Vector3d xyz = Vector3d(
				randrange(-1.0, 1.0),
				randrange(-1.0, 1.0),
				randrange(-1.0, 1.0)
			);
		xyz /= xyz.norm();
		xyz[0] *= x_size;
		xyz[1] *= y_size;
		xyz[2] *= z_size;
		pts.col(i) = xyz;
	}
	rm.publishPointCloud(pts, {"test_pc"});
	printf("Published point cloud with %d points.\n", n_points);

	Affine3d tf_box;
	tf_box.setIdentity();
	tf_box.translation()[1] = y_size*2;
	rm.publishGeometry(DrakeShapes::Box( {x_size, y_size, z_size} ), 
					   tf_box,
					   {"test_box"});

	Affine3d tf_sphere;
	tf_sphere.setIdentity();
	tf_sphere.translation()[1] = y_size*2 *2;
	rm.publishGeometry(DrakeShapes::Sphere( y_size ), 
					   tf_sphere,
					   {"test_sphere"});

	Affine3d tf_cylinder;
	tf_cylinder.setIdentity();
	tf_cylinder.translation()[1] = y_size*2 *3;
	rm.publishGeometry(DrakeShapes::Cylinder( y_size, z_size ), 
					   tf_cylinder,
					   {"test_cylinder"});

	Affine3d tf_capsule;
	tf_capsule.setIdentity();
	tf_capsule.translation()[1] = y_size*2 *4;
	rm.publishGeometry(DrakeShapes::Capsule( y_size, z_size ), 
					   tf_capsule,
					   {"test_capsule"});

	RigidBodyTree<double> tree;
	AddModelInstanceFromUrdfFileWithRpyJointToWorld("/home/gizatt/object_detection/drake/drake/examples/kuka_iiwa_arm/urdf/iiwa14.urdf", &tree);
	Affine3d tf_robot;
	tf_robot.setIdentity();
	tf_robot.translation()[1] = -y_size*2;
	rm.publishRigidBodyTree(tree, VectorXd::Zero(tree.get_num_positions()), {"test_robot"});

	return 0;
}