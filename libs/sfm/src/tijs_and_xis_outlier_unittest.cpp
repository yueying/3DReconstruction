#include <iostream>
#include <vector>

#include "testing.h"
#include "fblib/math/numeric.h"

#include "fblib/multiview/projection.h"
#include "fblib/multiview/nview_data_sets.h"
#include "fblib/sfm/linear_programming_interface.h"
#include "fblib/sfm/linear_programming_osi.h"
#include "fblib/sfm/bisection_linear_programming.h"
#include "fblib/sfm/tijs_and_xis_from_xi_ri_noise.h"

using namespace fblib::math;
using namespace fblib::multiview;
using namespace fblib::sfm;

TEST(Translation_Structure_L_Infinity_Noisy, Outlier_OSICLP_SOLVER) {

  const int view_num = 5;
  const int point_num = 5;
  const double focal_value = 1000;
  const double cx = 500, cy = 500;

  const NViewDataSet d =
    //NRealisticCamerasRing(view_num, point_num,
    NRealisticCamerasCardioid(view_num, point_num,
    NViewDatasetConfigurator(focal_value,focal_value,cx,cy,5,0));

  d.ExportToPLY("test_Before_Infinity.ply");
  // 对所有点进行三角化测试
  NViewDataSet d2 = d;

  //-- Set to 0 the future computed data to be sure of computation results :
  d2.point_3d_.fill(0); //Set _Xi of dataset 2 to 0 to be sure of new data computation
  fill(d2.translation_vector_.begin(), d2.translation_vector_.end(), Vec3(0.0,0.0,0.0));

  {
    // Prepare Rotation matrix (premultiplied by K)
    // Apply the K matrix to the rotation
    Mat3 K;
    K << focal_value, 0, cx,
      0, focal_value, cy,
      0, 0, 1;
    std::vector<Mat3> vec_KRotation(view_num);
    for (size_t i = 0; i < view_num; ++i) {
      vec_KRotation[i] = K * d.rotation_matrix_[i];
    }

    //set two point as outlier
    d2.projected_points_[0].col(0)(0) +=10; //Camera 0, measurement 0, noise on X coord
    d2.projected_points_[3].col(3)(1) -=8;  //Camera 3, measurement 3, noise on Y coord

    //Create the mega matrix
    Mat megaMat(4, d.actual_camera_num_*d.projected_points_[0].cols());
    {
      int cpt = 0;
      for (int i=0; i<d.actual_camera_num_;++i)
      {
        const int camIndex = i;
        for (int j=0; j<d.projected_points_[0].cols(); ++j)
        {
          megaMat(0,cpt) = d2.projected_points_[camIndex].col(j)(0);
          megaMat(1,cpt) = d2.projected_points_[camIndex].col(j)(1);
          megaMat(2,cpt) = j;
          megaMat(3,cpt) = camIndex;
          cpt++;
        }
      }
    }

    double admissibleResidual = 1.0/focal_value;
    std::vector<double> vec_solution((view_num + point_num + megaMat.cols())*3);
    OSI_CLP_SolverWrapper wrapper_osiclp_solver(vec_solution.size());
    TiXi_withNoise_L1_ConstraintBuilder constraint_builder( vec_KRotation, megaMat);
    std::cout << std::endl << "Bisection returns : " << std::endl;
    std::cout<< BisectionLinearProgramming<TiXi_withNoise_L1_ConstraintBuilder, LP_Constraints_Sparse>(
            wrapper_osiclp_solver,
            constraint_builder,
            &vec_solution,
            admissibleResidual,
            0.0, 1e-8);

    std::cout << "Found solution:\n";
    std::copy(vec_solution.begin(), vec_solution.end(), std::ostream_iterator<double>(std::cout, " "));

    //-- First the ti and after the Xi :

    //-- fill the ti
    for (int i=0; i < view_num; ++i)  {
      d2.translation_vector_[i] = K.inverse() * Vec3(vec_solution[3*i], vec_solution[3*i+1], vec_solution[3*i+2]);
      // Change Ci to -Ri*Ci
      d2.camera_center_[i] = -d2.rotation_matrix_[i].inverse() * d2.translation_vector_[i];
    }

    for (int i=0; i < point_num; ++i)  {
      size_t index = 3*view_num;
      d2.point_3d_.col(i) = Vec3(vec_solution[index+i*3], vec_solution[index+i*3+1], vec_solution[index+i*3+2]);
    }

    // Compute residuals L2 from estimated parameter values :
    std::cout << std::endl << "Residual : " << std::endl;
    Vec2 xk;
    double xsum = 0.0;
    for (int i = 0; i < d2.actual_camera_num_; ++i) {
        std::cout << "\nCamera : " << i << " \t:";
        for(int k = 0; k < d.projected_points_[0].cols(); ++k)
        {
          xk = Project(d2.P(i),  Vec3(d2.point_3d_.col(k)));
          double residual = (xk - d2.projected_points_[i].col(k)).norm();
          std::cout << Vec2(( xk - d2.projected_points_[i].col(k)).array().pow(2)).array().sqrt().mean() <<"\t";
          //-- Check that were measurement are not noisy the residual is small
          //  check were the measurement are noisy, the residual is important
          //if ((i != 0 && k != 0) || (i!=3 && k !=3))
          if ((i != 0 && k != 0) && (i!=3 && k !=3)) {
            EXPECT_NEAR(0.0, residual, 1e-6);
            xsum += residual;
          }
        }
        std::cout << std::endl;
    }
    double dResidual = xsum / (d2.actual_camera_num_*d.projected_points_[0].cols());
    std::cout << std::endl << "Residual mean in not noisy measurement: " << dResidual << std::endl;
    // Check that 2D re-projection and 3D point are near to GT.
    EXPECT_NEAR(0.0, dResidual, 1e-1);
  }

  d2.ExportToPLY("test_After_Infinity.ply");
}

