#ifndef FBLIB_SFM_RESECTION_KERNEL_H_
#define FBLIB_SFM_RESECTION_KERNEL_H_

#include <vector>
#include "fblib/multiview/projection.h"
#include "fblib/multiview/two_view_kernel.h"
#include "fblib/math/numeric.h"

using namespace fblib::multiview;
using namespace fblib::math;

namespace fblib {
	namespace sfm {
		
			/**
			 * Six-point resection
			 * P Matrix estimation (Pose estimation)
			 * Rely on L1 Resection algorithm.
			 * Work from 6 to N points.
			 */
			struct l1SixPointResectionSolver {
				enum { MINIMUM_SAMPLES = 6 };
				enum { MAX_MODELS = 1 };
				// Solve the problem of camera pose.
				// First 3d point will be translated in order to have X0 = (0,0,0,1)
				static void Solve(const Mat &point_2d, const Mat &point_3d, std::vector<Mat34> *P);

				// Compute the residual of the projection distance(point_2d, Project(P,point_3d))
				static double Error(const Mat34 & P, const Vec2 & point_2d, const Vec3 & point_3d)
				{
					Vec2 x = Project(P, point_3d);
					return (x - point_2d).norm();
				}
			};

			//-- Usable solver for the l1 6pt Resection Estimation
			typedef two_view::Kernel<l1SixPointResectionSolver,
				l1SixPointResectionSolver, Mat34>  l1PoseResectionKernel;

	}  // namespace sfm
}  // namespace fblib

#endif  // FBLIB_SFM_RESECTION_KERNEL_H_
