#include <iostream>
#include <vector>
#include "testing.h"

#include "mvg/feature/estimator_max_consensus.h"
#include "mvg/feature/score_evaluator.h"
#include "mvg/sfm/resection_kernel.h"

#include "mvg/camera/projection.h"
#include "mvg/multiview/nview_data_sets.h"

using namespace mvg::feature;
using namespace mvg::multiview;
using namespace mvg::camera;

TEST(Resection_L_Infinity, Robust_OutlierFree) {

  const int kViewNum = 3;
  const int kPointsNum = 10;
  const NViewDataSet d = NRealisticCamerasRing(kViewNum, kPointsNum,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // 假设相机内参矩阵为单位阵

  // 设定一个新的数据集，用于保存新计算的结果
  NViewDataSet d2 = d;

  const int kResectionCameraIndex = 2;
  // 将数据集中数据初始为0，保证计算出的值即为所求值
  d2.rotation_matrix_[kResectionCameraIndex] = Mat3::Zero();
  d2.translation_vector_[kResectionCameraIndex] = Vec3::Zero();

  {
    typedef  mvg::sfm::l1PoseResectionKernel KernelType;
    const Mat & point_2d = d2.projected_points_[kResectionCameraIndex];
    const Mat & point_3d = d2.point_3d_;
    KernelType kernel(point_2d, point_3d);
    ScorerEvaluator<KernelType> scorer(2*Square(0.6));
    Mat34 P = MaxConsensus(kernel, scorer, NULL, 128);

    // Check that Projection matrix is near to the GT :
    Mat34 GT_ProjectionMatrix = d.P(kResectionCameraIndex).array()
                                / d.P(kResectionCameraIndex).norm();
    Mat34 COMPUTED_ProjectionMatrix = P.array() / P.norm();

    // Extract K[R|t]
    Mat3 R,K;
    Vec3 t;
    KRt_From_P(P, &K, &R, &t);

    d2.rotation_matrix_[kResectionCameraIndex] = R;
    d2.translation_vector_[kResectionCameraIndex] = t;

    //CHeck matrix to GT, and residual
    EXPECT_NEAR( 0.0, FrobeniusDistance(GT_ProjectionMatrix, COMPUTED_ProjectionMatrix), 1e-2 );
    Mat pt4D = VStack(point_3d, Mat(Vec::Ones(point_3d.cols()).transpose()));
    EXPECT_NEAR( 0.0, RootMeanSquareError(point_2d, pt4D, COMPUTED_ProjectionMatrix), 1e-2);
  }
}

TEST(Resection_L_Infinity, Robust_OneOutlier) {

  const int kViewNum = 3;
  const int kPointsNum = 20;
  const NViewDataSet d = NRealisticCamerasRing(kViewNum, kPointsNum,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // 假设相机内参矩阵为单位阵

  d.ExportToPLY("test_Before_Infinity.ply");
  // 设定一个新的数据集，用于保存新计算的结果
  NViewDataSet d2 = d;

  const int kResectionCameraIndex = 2;
  // 将数据集中数据初始为0，保证计算出的值即为所求值
  d2.rotation_matrix_[kResectionCameraIndex] = Mat3::Zero();
  d2.translation_vector_[kResectionCameraIndex] = Vec3::Zero();

  //设置20%的点为局外点
  const int outlier_num = kPointsNum*0.2;
  for (int i=0; i < outlier_num; ++i)
  {
    d2.point_3d_.col(i)(0) += 120.0;
    d2.point_3d_.col(i)(1) += -60.0;
    d2.point_3d_.col(i)(2) += 80.0;
  }

  // Solve the problem and check that fitted value are good enough
  {
    typedef  mvg::sfm::l1PoseResectionKernel KernelType;
    const Mat & point_2d = d2.projected_points_[kResectionCameraIndex];
    const Mat & point_3d = d2.point_3d_;
    KernelType kernel(point_2d, point_3d);
    ScorerEvaluator<KernelType> scorer(Square(0.1)); //Highly intolerant for the test
    Mat34 P = MaxConsensus(kernel, scorer, NULL, 128);

    // Check that Projection matrix is near to the GT :
    Mat34 GT_ProjectionMatrix = d.P(kResectionCameraIndex).array()
      / d.P(kResectionCameraIndex).norm();
    Mat34 COMPUTED_ProjectionMatrix = P.array() / P.norm();

    // Extract K[R|t]
    Mat3 R,K;
    Vec3 t;
    KRt_From_P(P, &K, &R, &t);

    d2.rotation_matrix_[kResectionCameraIndex] = R;
    d2.translation_vector_[kResectionCameraIndex] = t;

    //CHeck matrix to GT, and residual
    EXPECT_NEAR( 0.0, FrobeniusDistance(GT_ProjectionMatrix, COMPUTED_ProjectionMatrix), 1e-3 );
    Mat pt4D = VStack(point_3d, Mat(Vec::Ones(point_3d.cols()).transpose()));
    EXPECT_NEAR( 0.0, RootMeanSquareError(point_2d, pt4D, COMPUTED_ProjectionMatrix), 1e-1);
  }
  d2.ExportToPLY("test_After_Infinity.ply");
}

