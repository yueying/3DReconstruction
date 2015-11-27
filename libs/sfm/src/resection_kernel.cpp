#include "sfm_precomp.h"
#include "mvg/sfm/resection.h"
#include "mvg/sfm/resection_kernel.h"

#include <cassert>

#include "mvg/sfm/linear_programming_interface.h"
#include "mvg/sfm/linear_programming_osi.h"
#include "mvg/sfm/bisection_linear_programming.h"

namespace mvg {
	namespace sfm {

		/**	对输入点进行平移操作
		 */
		void Translate(const Mat3X&input_points, const Vec3 &vec_translation,
			Mat3X *output_points) {
			output_points->resize(input_points.rows(), input_points.cols());
			for (size_t i = 0; i < input_points.cols(); ++i)  {
				output_points->col(i) = input_points.col(i) + vec_translation;
			}
		}

		void l1SixPointResectionSolver::Solve(const Mat &point_2d, const Mat &point_3d, std::vector<Mat34> *Ps) {

			assert(2 == point_2d.rows());
			assert(3 == point_3d.rows());
			assert(6 <= point_2d.cols());//保证点数大于等于6
			assert(point_2d.cols() == point_3d.cols());

			//-- Translate 3D points in order to have X0 = (0,0,0,1).
			Vec3 vec_translation = -point_3d.col(0);
			Mat4 translation_matrix = Mat4::Identity();
			translation_matrix.block<3, 1>(0, 3) = vec_translation;

			Mat3X output_points;
			Translate(point_3d, vec_translation, &output_points);

			std::vector<double> vec_solution(11);
			OSI_CLP_SolverWrapper wrapper_lp_solver(vec_solution.size());
			Resection_L1_ConstraintBuilder constraint_builder(point_2d, output_points);
			if (
				(BisectionLinearProgramming<Resection_L1_ConstraintBuilder, LP_Constraints_Sparse>(
				wrapper_lp_solver,
				constraint_builder,
				&vec_solution,
				1.0,
				0.0))
				)
			{
				// Move computed value to dataset for residual estimation.
				Mat34 P;
				P << vec_solution[0], vec_solution[1], vec_solution[2], vec_solution[3],
					vec_solution[4], vec_solution[5], vec_solution[6], vec_solution[7],
					vec_solution[8], vec_solution[9], vec_solution[10], 1.0;
				P = P * translation_matrix;
				Ps->push_back(P);
			}
		}

	}  // namespace sfm
}  // namespace mvg
