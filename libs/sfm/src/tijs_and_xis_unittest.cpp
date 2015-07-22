#include "fblib/multiview/nview_data_sets.h"
#include "fblib/math/numeric.h"
#include "testing.h"

#include "fblib/multiview/projection.h"

#include "fblib/sfm/linear_programming_interface.h"
#include "fblib/sfm/linear_programming_osi.h"

#include "fblib/sfm/bisection_linear_programming.h"
#include "fblib/sfm/tijs_and_xis_from_xi_ri.h"

#include <iostream>
#include <vector>

using namespace fblib::math;

using namespace fblib::multiview;
using namespace fblib::sfm;

TEST(Translation_Structure_L_Infinity, OSICLP_SOLVER) {

  const size_t kViewNum = 3;
  const size_t kPointsNum = 6;
  const NViewDataSet d = NRealisticCamerasRing(kViewNum, kPointsNum,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // 假设相机内参为单位阵

  d.ExportToPLY("test_Before_Infinity.ply");
  // 对所有点进行三角化测试
  NViewDataSet d2 = d;

  //-- Set to 0 the future computed data to be sure of computation results :
  d2.point_3d_.fill(0); //Set _Xi of dataset 2 to 0 to be sure of new data computation
  std::fill(d2.translation_vector_.begin(), d2.translation_vector_.end(), Vec3(0.0,0.0,0.0));

  //Create the mega matrix
  Mat megaMat(4, d.actual_camera_num_*d.projected_points_[0].cols());
  {
    size_t cpt = 0;
    for (size_t i=0; i<d.actual_camera_num_;++i)
    {
      const size_t camera_index = i;
      for (size_t j=0; j < d.projected_points_[0].cols(); ++j)
      {
        megaMat(0,cpt) = d.projected_points_[camera_index].col(j)(0);
        megaMat(1,cpt) = d.projected_points_[camera_index].col(j)(1);
        megaMat(2,cpt) = j;
        megaMat(3,cpt) = camera_index;
        cpt++;
      }
    }
  }

  // Solve the problem and check that fitted value are good enough
  {
    std::vector<double> vec_solution((kViewNum + kPointsNum)*3);

    OSI_CLP_SolverWrapper wrapper_osiclp_solver(vec_solution.size());
    Translation_Structure_L1_ConstraintBuilder constraint_builder( d.rotation_matrix_, megaMat);
    EXPECT_TRUE(
      (BisectionLinearProgramming<Translation_Structure_L1_ConstraintBuilder, LP_Constraints_Sparse>(
      wrapper_osiclp_solver,
      constraint_builder,
      &vec_solution,
      1.0,
      0.0))
    );

    // Move computed value to dataset for residual estimation.
    {
      //-- fill the ti
      for (size_t i=0; i < kViewNum; ++i)
      {
        size_t index = i*3;
        d2.translation_vector_[i] = Vec3(vec_solution[index], vec_solution[index+1], vec_solution[index+2]);
        // Change Ci to -Ri*Ci
        d2.camera_center_[i] = -d2.rotation_matrix_[i] * d2.translation_vector_[i];
      }

      //-- Now the Xi :
      for (size_t i=0; i < kPointsNum; ++i) {
        size_t index = 3*kViewNum;
        d2.point_3d_.col(i) = Vec3(vec_solution[index+i*3], vec_solution[index+i*3+1], vec_solution[index+i*3+2]);
      }
    }

    // Compute residuals L2 from estimated parameter values :
    Vec2 xk, xsum(0.0,0.0);
    for (size_t i = 0; i < d2.actual_camera_num_; ++i) {
      for(size_t k = 0; k < d.projected_points_[0].cols(); ++k)
      {
        xk = Project(d2.P(i), Vec3(d2.point_3d_.col(k)));
        xsum += Vec2(( xk - d2.projected_points_[i].col(k)).array().pow(2));
      }
    }
    double dResidual2D = (xsum.array().sqrt().sum());

    // Check that 2D re-projection and 3D point are near to GT.
    EXPECT_NEAR(0.0, dResidual2D, 1e-4);
  }

  d2.ExportToPLY("test_After_Infinity.ply");
}

TEST(Translation_Structure_L_Infinity, OSICLP_SOLVER_K) {

  const size_t kViewNum = 3;
  const size_t kPointsNum = 6;
  const NViewDataSet d = NRealisticCamerasRing(kViewNum, kPointsNum,
    NViewDatasetConfigurator(1000,1000,500,500,5,0));

  d.ExportToPLY("test_Before_Infinity.ply");
  // 对所有点进行三角化测试
  NViewDataSet d2 = d;

  //-- Set to 0 the future computed data to be sure of computation results :
  d2.point_3d_.fill(0); //Set _Xi of dataset 2 to 0 to be sure of new data computation
  std::fill(d2.translation_vector_.begin(), d2.translation_vector_.end(), Vec3(0.0,0.0,0.0));

  //Create the mega matrix
  Mat megaMat(4, d.actual_camera_num_*d.projected_points_[0].cols());
  {
    size_t cpt = 0;
    for (size_t i=0; i < d.actual_camera_num_;++i)
    {
      const size_t camera_index = i;
      for (size_t j=0; j < (size_t)d.projected_points_[0].cols(); ++j)
      {
        megaMat(0,cpt) = d.projected_points_[camera_index].col(j)(0);
        megaMat(1,cpt) = d.projected_points_[camera_index].col(j)(1);
        megaMat(2,cpt) = j;
        megaMat(3,cpt) = camera_index;
        cpt++;
      }
    }
  }

  // Solve the problem and check that fitted value are good enough
  {
    std::vector<double> vec_solution((kViewNum + kPointsNum)*3);

    std::vector<Mat3> vec_KR(d.rotation_matrix_);
    for(int i=0;i < kViewNum; ++i)
      vec_KR[i] = d.camera_matrix_[0] * d.rotation_matrix_[i];

    OSI_CLP_SolverWrapper wrapper_osiclp_solver(vec_solution.size());
    Translation_Structure_L1_ConstraintBuilder constraint_builder( vec_KR, megaMat);
    EXPECT_TRUE(
      (BisectionLinearProgramming<Translation_Structure_L1_ConstraintBuilder, LP_Constraints_Sparse>(
      wrapper_osiclp_solver,
      constraint_builder,
      &vec_solution,
      1.0,
      0.0))
    );
    
    // Move computed value to dataset for residual estimation.
    {
      //-- fill the ti
      for (size_t i=0; i < kViewNum; ++i)
      {
        size_t index = i*3;
        d2.translation_vector_[i] = d.camera_matrix_[0].inverse() * Vec3(vec_solution[index], vec_solution[index+1], vec_solution[index+2]);
        // Change Ci to -Ri*Ci
        d2.camera_center_[i] = -d2.rotation_matrix_[i] * d2.translation_vector_[i];
      }

      //-- Now the Xi :
      for (size_t i=0; i < kPointsNum; ++i) {
        size_t index = 3*kViewNum;
        d2.point_3d_.col(i) = Vec3(vec_solution[index+i*3], vec_solution[index+i*3+1], vec_solution[index+i*3+2]);
      }
    }

    // Compute residuals L2 from estimated parameter values :
    Vec2 xk, xsum(0.0,0.0);
    for (size_t i = 0; i < d2.actual_camera_num_; ++i) {
      for(size_t k = 0; k < (size_t)d.projected_points_[0].cols(); ++k)
      {
        xk = Project(d2.P(i), Vec3(d2.point_3d_.col(k)));
        xsum += Vec2(( xk - d2.projected_points_[i].col(k)).array().pow(2));
      }
    }
    double dResidual2D = (xsum.array().sqrt().sum());

    // Check that 2D re-projection and 3D point are near to GT.
    EXPECT_NEAR(0.0, dResidual2D, 1e-4);
  }

  d2.ExportToPLY("test_After_Infinity.ply");
}

