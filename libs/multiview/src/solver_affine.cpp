#include "multiview_precomp.h"
#include "mvg/multiview/solver_affine.h"

namespace mvg {
	namespace multiview{
		
		bool Affine2DFromCorrespondencesLinear(const Mat &x1, const Mat &x2,
			Mat3 *M,
			double expected_precision) {
			// 确保坐标为欧式坐标，点的个数大于3
			assert(2 == x1.rows());
			assert(3 <= x1.cols());
			assert(x1.rows() == x2.rows());
			assert(x1.cols() == x2.cols());

			// 计算A x = B
			const Mat::Index n = x1.cols();
			Mat A = Mat::Zero(2 * n, 6);
			Mat b = Mat::Zero(2 * n, 1);
			for (Mat::Index i = 0; i < n; ++i) {
				const Mat::Index j = i * 2;
				A(j, 0) = x1(0, i);
				A(j, 1) = x1(1, i);
				A(j, 4) = 1.0;

				A(j + 1, 2) = x1(0, i);
				A(j + 1, 3) = x1(1, i);
				A(j + 1, 5) = 1.0;

				b(j, 0) = x2(0, i);
				b(j + 1, 0) = x2(1, i);
			}
			// 计算
			Vec x = A.fullPivLu().solve(b);//通过LU分解解算器
			if ((A * x).isApprox(b, expected_precision))  {
				*M << x(0), x(1), x(4),
					x(2), x(3), x(5),
					0.0, 0.0, 1.0;
				return true;
			}
			else {
				return false;
			}
		}

		
		bool Affine3DFromCorrespondencesLinear(const Mat &x1,
			const Mat &x2,
			Mat4 *M,
			double expected_precision) {
			assert(3 == x1.rows());
			assert(4 <= x1.cols());
			assert(x1.rows() == x2.rows());
			assert(x1.cols() == x2.cols());

			const Mat::Index n = x1.cols();
			Mat A = Mat::Zero(3 * n, 12);
			Mat b = Mat::Zero(3 * n, 1);
			for (Mat::Index i = 0; i < n; ++i) {
				const Mat::Index j = i * 3;
				const Mat::Index j1 = j + 1;
				const Mat::Index j2 = j + 2;
				A(j, 0) = x1(0, i);
				A(j, 1) = x1(1, i);
				A(j, 2) = x1(2, i);
				A(j, 9) = 1.0;

				A(j1, 3) = x1(0, i);
				A(j1, 4) = x1(1, i);
				A(j1, 5) = x1(2, i);
				A(j1, 10) = 1.0;

				A(j2, 6) = x1(0, i);
				A(j2, 7) = x1(1, i);
				A(j2, 8) = x1(2, i);
				A(j2, 11) = 1.0;

				b(j, 0) = x2(0, i);
				b(j1, 0) = x2(1, i);
				b(j2, 0) = x2(2, i);
			}
			// 计算 A x = B
			Vec x = A.fullPivLu().solve(b);
			if ((A * x).isApprox(b, expected_precision))  {
				*M << x(0), x(1), x(2), x(9), // a b c x
					x(3), x(4), x(5), x(10), // d e f y
					x(6), x(7), x(8), x(11), // g h i z
					0.0, 0.0, 0.0, 1.0;
				return true;
			}
			else {
				return false;
			}
		}
	}// namespace multiview
} // namespace mvg
