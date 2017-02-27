#include "RemoteTreeViewerWrapper.hpp"
#include "../common/common.hpp"

using namespace Eigen;

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
	printf("Inside!\n");
	return 0;
}