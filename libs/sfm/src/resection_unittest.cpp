#include "testing.h"

#include <iostream>
#include <vector>

#include "mvg/multiview/nview_data_sets.h"
#include "mvg/camera/projection.h"
#include "mvg/sfm/linear_programming_interface.h"
#include "mvg/sfm/linear_programming_osi.h"
#include "mvg/sfm/bisection_linear_programming.h"
#include "mvg/sfm/resection.h"

using namespace mvg::math;
using namespace mvg::multiview;
using namespace mvg::sfm;
using namespace mvg::camera;

// 平移操作
void Translate(const Mat3X &input_3d_point, const Vec3 &translation,
  Mat3X * output_3d_point)  {
    output_3d_point->resize(input_3d_point.rows(), input_3d_point.cols());
    for (size_t i=0; i<(size_t)input_3d_point.cols(); ++i) {
      output_3d_point->col(i) = input_3d_point.col(i) + translation;
    }
}

TEST(Resection_L_Infinity, OSICLP) {

  const int kViewNum = 3;
  const int kPointsNum = 10;
  const NViewDataSet d = NRealisticCamerasRing(kViewNum, kPointsNum,
    NViewDatasetConfigurator(1,1,0,0,5,0));//配置内参信息

  d.ExportToPLY("test_Before_Infinity.ply");
  // 设置数据集，将值设0
  NViewDataSet d2 = d;

  const int resection_camera_index = 2;
  // 设置为0，保证将来的值即为计算出来的值
  d2.rotation_matrix_[resection_camera_index] = Mat3::Zero();
  d2.translation_vector_[resection_camera_index] = Vec3::Zero();

  // Solve the problem and check that fitted value are good enough
  {
    std::vector<double> vec_solution(11);

    //-- Translate 3D point in order to have X0 = (0,0,0,1).
    Vec3 translation = - d2.point_3d_.col(0);
    Mat4 translation_matrix = Mat4::Identity();
    translation_matrix << 1, 0, 0, translation(0),
                         0, 1, 0, translation(1),
                         0, 0, 1, translation(2),
                         0, 0, 0, 1;
    Mat3X output_3d_point;
    Translate(d2.point_3d_, translation, &output_3d_point);

    OSI_CLP_SolverWrapper wrapper_osiclp_lp_solver(vec_solution.size());
    Resection_L1_ConstraintBuilder constraint_builder(d2.projected_points_[resection_camera_index], output_3d_point);
    EXPECT_TRUE(
      (BisectionLinearProgramming<Resection_L1_ConstraintBuilder, LP_Constraints_Sparse>(
      wrapper_osiclp_lp_solver,
      constraint_builder,
      &vec_solution,
      1.0,
      0.0))
    );

    // Move computed value to dataset for residual estimation.
    Mat34 P;
    P << vec_solution[0], vec_solution[1], vec_solution[2], vec_solution[3],
         vec_solution[4], vec_solution[5], vec_solution[6], vec_solution[7],
         vec_solution[8], vec_solution[9], vec_solution[10], 1.0;
    P = P * translation_matrix;

    // Check that Projection matrix is near to the GT :
    Mat34 gt_projection_matrix = d.P(resection_camera_index).array()
                                / d.P(resection_camera_index).norm();
    Mat34 computed_projection_matrix = P.array() / P.norm();
    EXPECT_MATRIX_NEAR(gt_projection_matrix, computed_projection_matrix, 1e-4);
  }
  d2.ExportToPLY("test_After_Infinity.ply");
}



