#ifndef FBLIB_MULTIVIEW_SOLVER_HOMOGRAPHY_KERNEL_H_
#define FBLIB_MULTIVIEW_SOLVER_HOMOGRAPHY_KERNEL_H_

#include <vector>
#include "fblib/multiview/projection.h"
#include "fblib/multiview/two_view_kernel.h"

using namespace fblib::math;

namespace fblib {
	namespace multiview {
		namespace homography {

			struct FourPointSolver {
				enum { MINIMUM_SAMPLES = 4 };
				enum { MAX_MODELS = 1 };
				/**
				 * 通过直接线性变换计算(DLT)计算两幅图像之间的单应关系
				 *
				 * \param x  A 2xN matrix of column vectors.
				 * \param y  A 2xN matrix of column vectors.
				 * \param Hs A vector into which the computed homography is stored.
				 *
				 * The estimated homography should approximatelly hold the condition y = H x.
				 */
				static void Solve(const Mat &x, const Mat &y, std::vector<Mat3> *Hs);
			};

			// Should be distributed as Chi-squared with k = 2.
			struct AsymmetricError {
				static double Error(const Mat &H, const Vec2 &x1, const Vec2 &x2) {
					Vec3 x2h_est = H * EuclideanToHomogeneous(x1);
					Vec2 x2_est = x2h_est.head<2>() / x2h_est[2];
					return (x2 - x2_est).squaredNorm();
				}
			};

			// Kernel that works on original data point
			typedef two_view::Kernel < FourPointSolver, AsymmetricError, Mat3 >
				UnnormalizedKernel;

			// By default use the normalized version for increased robustness.
			typedef two_view::Kernel <
				two_view::NormalizedSolver<FourPointSolver, UnnormalizerI>,
				AsymmetricError,
				Mat3 >
				Kernel;

		}  // namespace homography
	}  // namespace multiview
}  // namespace fblib

#endif // FBLIB_MULTIVIEW_SOLVER_HOMOGRAPHY_KERNEL_H_
