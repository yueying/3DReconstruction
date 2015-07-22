#ifndef FBLIB_SFM_TRIANGULATION_H_
#define FBLIB_SFM_TRIANGULATION_H_

#include <utility>
#include <vector>

#include "fblib/math/numeric.h"
#include "fblib/sfm/linear_programming_interface.h"

//- 参考论文
//- [1] "Multiple-View Geometry under the L_\infty Norm."
//- Author : Fredrik Kahl, Richard Hartley.
//- Date : 9 sept 2008.

//- [2] "Multiple View Geometry and the L_\infty -norm"
//- Author : Fredrik Kahl
//- ICCV 2005.

namespace fblib   {
	namespace sfm  {


		//-- Triangulation
		//    - Estimation of X from vec_projection_matrix and xij
		// [1] -> 5.1 The triangulation problem
		//
		//    - This implementation Use L1 norm instead of the L2 norm of
		//      the paper, it allows to use standard standard LP
		//      (simplex) instead of using SOCP (second order cone programming).
		//      Implementation by Pierre Moulon
		//

		void EncodeTriangulation(const std::vector<Mat34> &vec_projection_matrix,
			const Mat2X & projection_x_ij,
			double gamma, // 设置上边界
			Mat & A, Vec & C)
		{
			// Build A, C matrix.

			const size_t camera_num = vec_projection_matrix.size();
			A.resize(5 * camera_num, 3);
			C.resize(5 * camera_num, 1);

			int cpt = 0;
			for (int i = 0; i < camera_num; ++i)
			{
				// 取得每幅图像对应的外参以及对应的投影点
				const Mat3 R = vec_projection_matrix[i].block<3, 3>(0, 0);
				const Vec3 t = vec_projection_matrix[i].block<3, 1>(0, 3);
				const Mat2X pt = projection_x_ij.col(i);

				// A (Rotational part):
				A.block<1, 3>(cpt, 0) = R.row(0) - pt(0) * R.row(2) - gamma * R.row(2);
				A.block<1, 3>(cpt + 1, 0) = R.row(1) - pt(1) * R.row(2) - gamma * R.row(2);
				A.block<1, 3>(cpt + 2, 0) = -R.row(2);
				A.block<1, 3>(cpt + 3, 0) = -R.row(0) + pt(0) * R.row(2) - gamma * R.row(2);
				A.block<1, 3>(cpt + 4, 0) = -R.row(1) + pt(1) * R.row(2) - gamma * R.row(2);

				// C (translation part):
				C(cpt) = -t(0) + pt(0) * t(2) + gamma * t(2);
				C(cpt + 1) = -t(1) + pt(1) * t(2) + gamma * t(2);
				C(cpt + 2) = t(2);
				C(cpt + 3) = t(0) - pt(0) * t(2) + gamma * t(2);
				C(cpt + 4) = t(1) - pt(1) * t(2) + gamma * t(2);

				//- Next entry
				cpt += 5;
			}
		}

		/// Kernel that set Linear constraints for the Triangulation Problem.
		///  Designed to be used with bisectionLP and LP_Solver interface.
		///
		/// Triangulation :
		///    - Estimation of Xi from Pj and xij
		/// Implementation of problem of [1] -> 5.1 The triangulation problem
		///  under a Linear Program form.
		// 
		struct Triangulation_L1_ConstraintBuilder
		{
			Triangulation_L1_ConstraintBuilder(
				const std::vector<Mat34> &vec_projection_matrix,
				const Mat2X &projection_x_ij)
			{
				vec_projection_matrix_ = vec_projection_matrix;
				projection_x_ij_ = projection_x_ij;
			}

			/**	将三角定位问题转线性约束问题，构建约束
			 */
			bool Build(double gamma, LP_Constraints &constraint)
			{
				EncodeTriangulation(vec_projection_matrix_, projection_x_ij_,
					gamma,
					constraint.constraint_mat_,
					constraint.constraint_num_);
				
				// 添加约束条件，约束中给出3个参数[X,Y,Z]，没有边界，约束符号(<=)
				constraint.parameter_num_ = 3;
				constraint.vec_bounds_ = std::vector< std::pair<double, double> >(1);
				std::fill(constraint.vec_bounds_.begin(), constraint.vec_bounds_.end(),
					std::make_pair((double)-1e+30, (double)1e+30));
				// 给出约束类型
				constraint.vec_constrained_type_.resize(constraint.constraint_mat_.rows());
				std::fill(constraint.vec_constrained_type_.begin(), constraint.vec_constrained_type_.end(),
					LP_Constraints::LP_LESS_OR_EQUAL);

				return true;
			}

			/// 用于三角定位的数据 :
			std::vector<Mat34> vec_projection_matrix_;  //!< 投影矩阵
			Mat2X projection_x_ij_;                 //!< 二维投影点 : xij = Pj*Xi
		};

	} // namespace sfm
} // namespace fblib

#endif // FBLIB_SFM_TRIANGULATION_H_
