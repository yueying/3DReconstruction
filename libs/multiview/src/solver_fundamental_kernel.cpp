#include "multiview_precomp.h"
#include "mvg/multiview/solver_fundamental_kernel.h"
#include "mvg/math/poly.h"
#include "mvg/math/numeric.h"
using namespace mvg::math;

namespace mvg {
	namespace multiview {
		namespace fundamental {

			void SevenPointSolver::Solve(const Mat &x1, const Mat &x2, std::vector<Mat3> *fundamental_matrix) {
				assert(2 == x1.rows());
				assert(7 <= x1.cols());
				assert(x1.rows() == x2.rows());
				assert(x1.cols() == x2.cols());

				Vec9 f1, f2;
				if (x1.cols() == 7) {
					// 将基础矩阵 x'T*F*x = 0.转成Af = 0
					typedef Eigen::Matrix<double, 9, 9> Mat9;
					// 构建约束方程
					Mat9 A = Mat::Zero(9, 9);
					EncodeEpipolarEquation(x1, x2, &A);
					// Find the two fundamental_matrix matrices in the nullspace of A.
					Nullspace2(&A, &f1, &f2);
				}
				else  {
					// Set up the homogeneous system Af = 0 from the equations x'T*fundamental_matrix*x = 0.
					Mat A(x1.cols(), 9);
					EncodeEpipolarEquation(x1, x2, &A);
					// Find the two fundamental_matrix matrices in the nullspace of A.
					Nullspace2(&A, &f1, &f2);
				}

				Mat3 F1 = Map<RMat3>(f1.data());
				Mat3 F2 = Map<RMat3>(f2.data());

				// Then, use the condition det(fundamental_matrix) = 0 to determine fundamental_matrix. In other words, solve
				// det(F1 + a*F2) = 0 for a.
				double a = F1(0, 0), j = F2(0, 0),
					b = F1(0, 1), k = F2(0, 1),
					c = F1(0, 2), l = F2(0, 2),
					d = F1(1, 0), m = F2(1, 0),
					e = F1(1, 1), n = F2(1, 1),
					f = F1(1, 2), o = F2(1, 2),
					g = F1(2, 0), p = F2(2, 0),
					h = F1(2, 1), q = F2(2, 1),
					i = F1(2, 2), r = F2(2, 2);

				// Run fundamental_7point_coeffs.py to get the below coefficients.
				// The coefficients are in ascending powers of alpha, i.e. P[N]*x^N.
				double P[4] = {
					a*e*i + b*f*g + c*d*h - a*f*h - b*d*i - c*e*g,
					a*e*r + a*i*n + b*f*p + b*g*o + c*d*q + c*h*m + d*h*l + e*i*j + f*g*k -
					a*f*q - a*h*o - b*d*r - b*i*m - c*e*p - c*g*n - d*i*k - e*g*l - f*h*j,
					a*n*r + b*o*p + c*m*q + d*l*q + e*j*r + f*k*p + g*k*o + h*l*m + i*j*n -
					a*o*q - b*m*r - c*n*p - d*k*r - e*l*p - f*j*q - g*l*n - h*j*o - i*k*m,
					j*n*r + k*o*p + l*m*q - j*o*q - k*m*r - l*n*p };

				// Solve for the roots of P[3]*x^3 + P[2]*x^2 + P[1]*x + P[0] = 0.
				double roots[3];
				int num_roots = SolveCubicPolynomial(P, roots);

				// Build the fundamental matrix for each solution.
				for (int kk = 0; kk < num_roots; ++kk)  {
					fundamental_matrix->push_back(F1 + roots[kk] * F2);
				}
			}

			void EightPointSolver::Solve(const Mat &x1, const Mat &x2, std::vector<Mat3> *Fs) {
				assert(2 == x1.rows());
				assert(8 <= x1.cols());
				assert(x1.rows() == x2.rows());
				assert(x1.cols() == x2.cols());

				Vec9 f;
				if (x1.cols() == 8) {
					typedef Eigen::Matrix<double, 9, 9> Mat9;
					// In the minimal solution use fixed sized matrix to let Eigen and the
					//  compiler doing the maximum of optimization.
					Mat9 A = Mat::Zero(9, 9);
					EncodeEpipolarEquation(x1, x2, &A);
					Nullspace(&A, &f);
				}
				else  {
					MatX9 A(x1.cols(), 9);
					EncodeEpipolarEquation(x1, x2, &A);
					Nullspace(&A, &f);
				}

				Mat3 fundamental_matrix = Map<RMat3>(f.data());

				// Force the fundamental property if the A matrix has full rank.
				// HZ 11.1.1 pag.280
				if (x1.cols() > 8) {
					// Force fundamental matrix to have rank 2
					Eigen::JacobiSVD<Mat3> USV(fundamental_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
					Vec3 d = USV.singularValues();
					d[2] = 0.0;
					fundamental_matrix = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();
				}
				Fs->push_back(fundamental_matrix);
			}

		}  // namespace fundamental
	}  // namespace multiview
}  // namespace mvg
