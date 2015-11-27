#ifndef MVG_SFM_RESECTION_H_
#define MVG_SFM_RESECTION_H_

#include <fstream>
#include <utility>
#include <vector>

#include "mvg/math/numeric.h"
#include "mvg/sfm/linear_programming_interface.h"
using namespace mvg::math;

// 算法实现参考：
//- Kahl F, Hartley R. Multiple-View Geometry Under the {$ L_infty $}-Norm[J].
//- Pattern Analysis and Machine Intelligence, IEEE Transactions on, 2008, 30(9): 1603-1617.

//- Kahl F. Multiple view geometry and the L∞-norm[C]
//- ICCV 2005. Tenth IEEE International Conference on. IEEE, 2005, 2: 1002-1009.


namespace mvg{
	namespace sfm{


		//-- Camera Resection
		//    - Estimation of [Ri|Ti] from xij and Xi
		// [1] -> 5.4 Camera Resectioning
		//
		//  The projection is parametrized as :
		//      | p11 p12 p13 p14 |
		//   P= | p21 p22 p23 p24 |
		//      | p31 p32 p33 1   |
		//
		//    - This implementation Use L1 norm instead of the L2 norm of
		//      the paper, it allows to use standard standard LP
		//      (simplex) instead of using SOCP (second order cone programming).
		//      Implementation by Pierre Moulon
		//
		static void EncodeResection(const Mat2X & point_2d,
			const Mat3X & point_3d,
			double gamma, // Start upper bound
			RSparseMat & A, Vec & C)
		{
			// Build Constraint matrix.

			const int Nobs = point_2d.cols();

			assert(point_2d.cols() == point_3d.cols());
			assert(point_2d.cols() >= 6 && "The problem requires at least 6 points");

			A.resize(Nobs * 5, 11);

			C.resize(Nobs * 5, 1);
			C.fill(0.0);

			for (int p = 0; p < Nobs; ++p)
			{
				const Vec2 pt2d = point_2d.col(p);
				const Vec3 point_3d = point_3d.col(p);

				// Compute and setup constraint :

				// Cheirality
				// R^3 X + t >0
				// - R^3 X - t < 0 => // - R^3 X < 1.0 because t3 = 1.0 by default
				int cpti = 8;
				int cptj = 5 * p;
				A.coeffRef(cptj, cpti) = -point_3d(0);
				A.coeffRef(cptj, cpti + 1) = -point_3d(1);
				A.coeffRef(cptj, cpti + 2) = -point_3d(2);
				C(cptj) = 1.0;

				// First constraint <= s
				// R1 * X + t1 + R3 * X *(-xij -s) <= xij t3 + s t3
				cpti = 0;
				cptj += 1;
				A.coeffRef(cptj, cpti) = point_3d(0);
				A.coeffRef(cptj, cpti + 1) = point_3d(1);
				A.coeffRef(cptj, cpti + 2) = point_3d(2);
				A.coeffRef(cptj, cpti + 3) = 1.0;
				cpti = 4;
				A.coeffRef(cptj + 1, cpti) = point_3d(0);
				A.coeffRef(cptj + 1, cpti + 1) = point_3d(1);
				A.coeffRef(cptj + 1, cpti + 2) = point_3d(2);
				A.coeffRef(cptj + 1, cpti + 3) = 1.0;

				cpti = 8;
				Mat temp;
				temp = Vec2((-pt2d).array() - gamma) * point_3d.transpose();
				A.coeffRef(cptj, cpti) = temp(0, 0);
				A.coeffRef(cptj, cpti + 1) = temp(0, 1);
				A.coeffRef(cptj, cpti + 2) = temp(0, 2);

				A.coeffRef(cptj + 1, cpti) = temp(1, 0);
				A.coeffRef(cptj + 1, cpti + 1) = temp(1, 1);
				A.coeffRef(cptj + 1, cpti + 2) = temp(1, 2);

				C(cptj) = gamma + pt2d(0);
				C(cptj + 1) = gamma + pt2d(1);


				// Second constraint >= s
				// -R1 * X - t1 + R3 * X *(xij -s) <= - xij t3 + s t3
				cpti = 0;
				cptj += 2;
				A.coeffRef(cptj, cpti) = -point_3d(0);
				A.coeffRef(cptj, cpti + 1) = -point_3d(1);
				A.coeffRef(cptj, cpti + 2) = -point_3d(2);
				A.coeffRef(cptj, cpti + 3) = -1.0;
				cpti = 4;
				A.coeffRef(cptj + 1, cpti) = -point_3d(0);
				A.coeffRef(cptj + 1, cpti + 1) = -point_3d(1);
				A.coeffRef(cptj + 1, cpti + 2) = -point_3d(2);
				A.coeffRef(cptj + 1, cpti + 3) = -1.0;

				cpti = 8;
				temp = Vec2(pt2d.array() - gamma) * point_3d.transpose();
				A.coeffRef(cptj, cpti) = temp(0, 0);
				A.coeffRef(cptj, cpti + 1) = temp(0, 1);
				A.coeffRef(cptj, cpti + 2) = temp(0, 2);

				A.coeffRef(cptj + 1, cpti) = temp(1, 0);
				A.coeffRef(cptj + 1, cpti + 1) = temp(1, 1);
				A.coeffRef(cptj + 1, cpti + 2) = temp(1, 2);

				C(cptj) = gamma - pt2d(0);
				C(cptj + 1) = gamma - pt2d(1);
			}
		}

		/// Kernel that set Linear constraints for the
		///   - Translation Registration and Structure Problem.
		///  Designed to be used with bisectionLP and LP_Solver interface.
		///
		/// Implementation of camera Resection
		///    - Estimation of [Ri|Ti] from xij and Xi
		/// [1] -> 5.4 Camera Resectioning
		///
		struct Resection_L1_ConstraintBuilder
		{
			Resection_L1_ConstraintBuilder(
				const Mat2X & point_2d,
				const Mat3X & point_3d)
			{
				point_2d_ = point_2d;
				point_3d_ = point_3d;
			}

			/// Setup constraints for the Resection problem,
			///  in the LP_Constraints object.
			bool Build(double gamma, LP_Constraints_Sparse & constraint)
			{
				EncodeResection(point_2d_, point_3d_,
					gamma,
					constraint.constraint_mat_,
					constraint.constraint_num_);

				//-- Setup additional information about the Linear Program constraint
				// We look for:
				// P = [P00 P01 P02 P03;
				//      P10 P11 P12 P13;
				//      P20 P21 P22 1.0];
				const int NParams = 4 * 2 + 3;

				constraint.parameter_num_ = NParams;
				constraint.vec_bounds_ = std::vector< std::pair<double, double> >(1);
				fill(constraint.vec_bounds_.begin(), constraint.vec_bounds_.end(),
					std::make_pair((double)-1e+30, (double)1e+30)); // lp_solve => getInfinite => DEF_INFINITE
				// Constraint sign are all LESS or equal (<=)
				constraint.vec_constrained_type_.resize(constraint.constraint_mat_.rows());
				fill(constraint.vec_constrained_type_.begin(), constraint.vec_constrained_type_.end(),
					LP_Constraints::LP_LESS_OR_EQUAL);

				return true;
			}

			Mat2X point_2d_;
			Mat3X point_3d_;
		};

	} // namespace sfm
} // namespace mvg

#endif // MVG_SFM_RESECTION_H_
