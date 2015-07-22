#include <cmath>
#include <cstdio>
#include <iostream>

#include "testing.h"

#include "fblib/multiview/nview_data_sets.h"
#include "fblib/multiview/projection.h"

#include "fblib/sfm/pinhole_ceres_functor.h"
#include "fblib/sfm/problem_data_container.h"

using namespace fblib::multiview;
using namespace fblib::sfm;

/**	通过Bundle Adjustment来优化相机的内外参，给出模拟环境，对观测点添加[-.5,.5]的随机噪声
 */
TEST(BundleAdjustment, EffectiveMinimization_RTf) {

	int nviews = 3;
	int npoints = 6;
	NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

	// 创建一个BA问题
	BAProblemData<7> ba_problem;

	// 对问题进行配置
	ba_problem.num_cameras_ = nviews;
	ba_problem.num_points_ = npoints;
	ba_problem.num_observations_ = nviews * npoints;

	ba_problem.point_index_.reserve(ba_problem.num_observations_);
	ba_problem.camera_index_.reserve(ba_problem.num_observations_);
	ba_problem.observations_.reserve(2 * ba_problem.num_observations_);

	ba_problem.num_parameters_ = 7 * ba_problem.num_cameras_ + 3 * ba_problem.num_points_;
	ba_problem.parameters_.reserve(ba_problem.num_parameters_);

	double ppx = 500, ppy = 500;
	// 填充数据
	for (int i = 0; i < npoints; ++i) {
		// 确定图像的每一帧投影点
		for (int j = 0; j < nviews; ++j) {
			ba_problem.camera_index_.push_back(j);
			ba_problem.point_index_.push_back(i);
			const Vec2 &pt = d.projected_points_[j].col(i);
			//添加 [-.5,.5] 的随机噪声
			ba_problem.observations_.push_back(pt(0) - ppx + rand() / RAND_MAX - .5);
			ba_problem.observations_.push_back(pt(1) - ppy + rand() / RAND_MAX - .5);
		}
	}

	// 添加相机参数 (R, t, focal)
	for (int j = 0; j < nviews; ++j) {
		// 旋转矩阵转欧拉角
		std::vector<double> angleAxis(3);
		ceres::RotationMatrixToAngleAxis((const double*)d.rotation_matrix_[j].data(), &angleAxis[0]);

		Vec3 t = d.translation_vector_[j];
		double focal = d.camera_matrix_[j](0, 0);
		ba_problem.parameters_.push_back(angleAxis[0]);
		ba_problem.parameters_.push_back(angleAxis[1]);
		ba_problem.parameters_.push_back(angleAxis[2]);
		ba_problem.parameters_.push_back(t[0]);
		ba_problem.parameters_.push_back(t[1]);
		ba_problem.parameters_.push_back(t[2]);
		ba_problem.parameters_.push_back(focal);
	}

	// 添加3D坐标点
	for (int i = 0; i < npoints; ++i) {
		Vec3 point_3d = d.point_3d_.col(i);
		ba_problem.parameters_.push_back(point_3d[0]);
		ba_problem.parameters_.push_back(point_3d[1]);
		ba_problem.parameters_.push_back(point_3d[2]);
	}

	// 对每次观测创建残差
	ceres::Problem problem;
	for (int i = 0; i < ba_problem.num_observations(); ++i) {
		// 3d点和相机作为输入，输出2维残差
		ceres::CostFunction* cost_function =
			new ceres::AutoDiffCostFunction<ErrorFuncRefineCamera3DPointsPinhole, 2, 7, 3>(
			new ErrorFuncRefineCamera3DPointsPinhole(
			&ba_problem.observations()[2 * i]));

		problem.AddResidualBlock(cost_function,
			NULL, // squared loss
			ba_problem.mutable_camera_for_observation(i),
			ba_problem.mutable_point_for_observation(i));
	}

	// 通过Ceres进行计算，这边可以考虑一下其它几种解法
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
		options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
		options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
	else
	{
		options.linear_solver_type = ceres::DENSE_SCHUR;
	}
	options.minimizer_progress_to_stdout = false;
	options.logging_type = ceres::SILENT;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";

	double residual_before = std::sqrt(summary.initial_cost / (ba_problem.num_observations_*2.));
	double residual_after = std::sqrt(summary.final_cost / (ba_problem.num_observations_*2.));

	std::cout << std::endl
		<< " Initial RMSE : " << residual_before << "\n"
		<< " Final RMSE : " << residual_after << "\n"
		<< std::endl;

	EXPECT_TRUE(summary.IsSolutionUsable());
	EXPECT_TRUE(residual_before > residual_after);
}


