﻿#include "multiview_precomp.h"
#include "fblib/multiview/solver_homography_kernel.h"

namespace fblib {
	namespace multiview {
		namespace homography {

			/// Setup the Direct Linear Transform.
			///  Use template in order to support fixed or dynamic sized matrix.
			/// Allow solve H as homogeneous(x2) = H homogeneous(x1)
			template<typename Matrix >
			void BuildActionMatrix(Matrix & L, const Mat &x, const Mat &y)  {

				const Mat::Index n = x.cols();
				for (int i = 0; i < n; ++i) {
					int j = 2 * i;
					L(j, 0) = x(0, i);
					L(j, 1) = x(1, i);
					L(j, 2) = 1.0;
					L(j, 6) = -y(0, i) * x(0, i);
					L(j, 7) = -y(0, i) * x(1, i);
					L(j, 8) = -y(0, i);

					++j;
					L(j, 3) = x(0, i);
					L(j, 4) = x(1, i);
					L(j, 5) = 1.0;
					L(j, 6) = -y(1, i) * x(0, i);
					L(j, 7) = -y(1, i) * x(1, i);
					L(j, 8) = -y(1, i);
				}
			}

			void FourPointSolver::Solve(const Mat &x, const Mat &y, std::vector<Mat3> *Hs) {
				assert(2 == x.rows());
				assert(4 <= x.cols());
				assert(x.rows() == y.rows());
				assert(x.cols() == y.cols());

				Mat::Index n = x.cols();

				Vec9 h;
				if (n == 4)  {
					// In the case of minimal configuration we use fixed sized matrix to let
					//  Eigen and the compiler doing the maximum of optimization.
					typedef Eigen::Matrix<double, 16, 9> Mat16_9;
					Mat16_9 L = Mat::Zero(16, 9);
					BuildActionMatrix(L, x, y);
					Nullspace(&L, &h);
				}
				else {
					MatX9 L = Mat::Zero(n * 2, 9);
					BuildActionMatrix(L, x, y);
					Nullspace(&L, &h);
				}
				Mat3 H = Map<RMat3>(h.data()); // map the linear vector as the H matrix
				Hs->push_back(H);
			}

		}  // namespace homography
	}  // namespace multiview
}  // namespace fblib
