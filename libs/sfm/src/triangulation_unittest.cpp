#include <iostream>
#include <vector>

#include "testing.h"
#include "fblib/math/numeric.h"
#include "fblib/multiview/nview_data_sets.h"
#include "fblib/multiview/projection.h"

#include "fblib/sfm/linear_programming_interface.h"
#include "fblib/sfm/linear_programming_osi.h"
#include "fblib/sfm/bisection_linear_programming.h"
#include "fblib/sfm/triangulation.h"

using namespace fblib::math;
using namespace fblib::multiview;
using namespace fblib::sfm;

TEST(lInfinityCV, Triangulation_OSICLPSOLVER) {

	NViewDataSet input_dataset = NRealisticCamerasRing(6, 10,
		NViewDatasetConfigurator(1, 1, 0, 0, 5, 0)); // 假设相机内参为单位阵

	std::vector<Mat34> vec_projection_matrix;// 对应的投影矩阵集

	input_dataset.ExportToPLY("test_Before_Infinity_Triangulation_OSICLP.ply");
	// 对所有点进行三角化测试
	NViewDataSet output_dataset = input_dataset;
	output_dataset.point_3d_.fill(0); // 将dataset 2中的3d点设置为0，以确保计算的值是新值

	for (size_t i = 0; i < input_dataset.actual_camera_num_; ++i)
		vec_projection_matrix.push_back(input_dataset.P(i));

	for (int k = 0; k < input_dataset.projected_points_[0].cols(); ++k)
	{
		Mat2X x_ij;
		x_ij.resize(2, input_dataset.actual_camera_num_);
		// 给出所有的对应点
		for (int i = 0; i < input_dataset.actual_camera_num_; ++i)
			x_ij.col(i) = input_dataset.projected_points_[i].col(k);

		std::vector<double> vec_solution(3);

		OSI_CLP_SolverWrapper wrapper_osiclp_solver(3);
		Triangulation_L1_ConstraintBuilder constraint_builder(vec_projection_matrix, x_ij);
		// Use bisection in order to find the global optimum and so find the
		//  best triangulated point under the L_infinity norm
		EXPECT_TRUE(
			(BisectionLinearProgramming<Triangulation_L1_ConstraintBuilder, LP_Constraints>(
			wrapper_osiclp_solver,constraint_builder,&vec_solution,1.0,0.0))
			);

		Vec3 XSolution(vec_solution[0], vec_solution[1], vec_solution[2]);
		output_dataset.point_3d_.col(k) = XSolution.transpose();

		// Compute residuals L2 from estimated parameter values :
		const Vec3 & X = XSolution;
		Vec2 x1, xsum(0.0, 0.0);
		for (int i = 0; i < output_dataset.actual_camera_num_; ++i) {
			x1 = Project(output_dataset.P(i), X);
			xsum += Vec2((x1 - x_ij.col(i)).array().pow(2));
		}
		double dResidual2D = (xsum.array().sqrt().sum());

		// Residual LInfinity between GT 3D point and found one
		double dResidual3D = DistanceLInfinity(XSolution, Vec3(input_dataset.point_3d_.col(k)));

		// Check that 2D re-projection and 3D point are near to GT.
		EXPECT_NEAR(0.0, dResidual2D, 1e-5);
		EXPECT_NEAR(0.0, dResidual3D, 1e-5);
	}
	output_dataset.ExportToPLY("test_After_Infinity_Triangulation_OSICLP.ply");
}
