#include "multiview_precomp.h"
#include "fblib/multiview/solver_essential_kernel.h"
#include "fblib/multiview/solver_fundamental_kernel.h"
#include "fblib/math/poly.h"
#include "fblib/multiview/solver_essential_five_point.h"
#include <cassert>

namespace fblib {
	namespace multiview {
		namespace essential {

			void PointWithNegativeDepth::Solve(const Mat &x1, const Mat &x2, vector<Mat3> *Es) {
				assert(2 == x1.rows());
				assert(8 <= x1.cols());
				assert(x1.rows() == x2.rows());
				assert(x1.cols() == x2.cols());

				MatX9 A(x1.cols(), 9);
				fundamental::EncodeEpipolarEquation(x1, x2, &A);

				Vec9 e;
				Nullspace(&A, &e);
				Mat3 E = Map<RMat3>(e.data());

				// Find the closest essential matrix to E in frobenius norm
				// E = UD'VT
				if (x1.cols() > 8) {
					Eigen::JacobiSVD<Mat3> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
					Vec3 d = USV.singularValues();
					double a = d[0];
					double b = d[1];
					d << (a + b) / 2., (a + b) / 2., 0.0;
					E = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();
				}
				Es->push_back(E);
			}

			void FivePointSolver::Solve(const Mat &x1, const Mat &x2, vector<Mat3> *E) {
				assert(2 == x1.rows());
				assert(5 <= x1.cols());
				assert(x1.rows() == x2.rows());
				assert(x1.cols() == x2.cols());

				FivePointsRelativePose(x1, x2, E);
			}

		}  // namespace essential
	}  // namespace multiview
}  // namespace fblib
