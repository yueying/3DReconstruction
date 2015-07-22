#include "fblib/multiview/essential.h"
#include "fblib/multiview/solver_essential_kernel.h"
#include "fblib/multiview/projection.h"
#include "testing.h"

#include "fblib/multiview/nview_data_sets.h"

using namespace fblib::multiview;

/// Check that the E matrix fit the Essential Matrix properties
/// Determinant is 0
///
#define EXPECT_ESSENTIAL_MATRIX_PROPERTIES(E, expectedPrecision) { \
  EXPECT_NEAR(0, E.determinant(), expectedPrecision); \
  Mat3 O = 2 * E * E.transpose() * E - (E * E.transpose()).trace() * E; \
  Mat3 zero3x3 = Mat3::Zero(); \
  EXPECT_MATRIX_NEAR(zero3x3, O, expectedPrecision);\
}

TEST(EightPointsRelativePose, EightPointsRelativePose_Kernel_IdFocal) {

  //-- Setup a circular camera rig and assert that 8PT relative pose works.
  const int iNviews = 5;
  NViewDataSet d = NRealisticCamerasRing(iNviews, 8,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  for(int i=0; i <iNviews; ++i)
  {
	  std::vector<Mat3> Es; // Essential,
	std::vector<Mat3> Rs;  // Rotation matrix.
	std::vector<Vec3> ts;  // Translation matrix.
    essential::PointWithNegativeDepth::Solve(d.projected_points_[i], d.projected_points_[(i+1)%iNviews], &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d.projected_points_[i].col(0);
      x2Col << d.projected_points_[(i+1)%iNviews].col(0);
	  EXPECT_TRUE(
        MotionFromEssentialAndCorrespondence(Es[s],
        d.camera_matrix_[i], x1Col,
        d.camera_matrix_[(i+1)%iNviews], x2Col,
        &Rs[s],
        &ts[s]));
    }
    //-- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d.rotation_matrix_[i], d.translation_vector_[i], d.rotation_matrix_[(i+1)%iNviews], d.translation_vector_[(i+1)%iNviews], &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {

      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-8);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    //-- Almost one solution must find the correct relative orientation
	EXPECT_TRUE(bsolution_found);
  }
}

TEST(EightPointsRelativePose, EightPointsRelativePose_Kernel) {

  typedef essential::EightPointKernel Kernel;

  int focal = 1000;
  int principal_Point = 500;

  //-- Setup a circular camera rig and assert that 8PT relative pose works.
  const int iNviews = 5;
  NViewDataSet d = NRealisticCamerasRing(iNviews, Kernel::MINIMUM_SAMPLES,
    NViewDatasetConfigurator(focal,focal,principal_Point,principal_Point,5,0)); // Suppose a camera with Unit matrix as K

  for(int i=0; i <iNviews; ++i)
  {
	  std::vector<Mat3> Es, Rs;  // Essential, Rotation matrix.
	  std::vector<Vec3> ts;      // Translation matrix.

    // Direct value do not work.
    // As we use reference, it cannot convert Mat2X& to Mat&
    Mat x0 = d.projected_points_[i];
    Mat x1 = d.projected_points_[(i+1)%iNviews];

    Kernel kernel(x0, x1, d.camera_matrix_[i], d.camera_matrix_[(i+1)%iNviews]);
	std::vector<size_t> samples;
    for (size_t k = 0; k < Kernel::MINIMUM_SAMPLES; ++k) {
      samples.push_back(k);
    }
    kernel.Fit(samples, &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d.projected_points_[i].col(0);
      x2Col << d.projected_points_[(i+1)%iNviews].col(0);
	  EXPECT_TRUE(
        MotionFromEssentialAndCorrespondence(Es[s],
        d.camera_matrix_[i], x1Col,
        d.camera_matrix_[(i+1)%iNviews], x2Col,
        &Rs[s],
        &ts[s]));
    }
    //-- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d.rotation_matrix_[i], d.translation_vector_[i], d.rotation_matrix_[(i+1)%iNviews], d.translation_vector_[(i+1)%iNviews], &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {

      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-8);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    //-- Almost one solution must find the correct relative orientation
	EXPECT_TRUE(bsolution_found);
  }
}

TEST(FivePointKernelTest, KernelError) {

  Mat x1(2, 5), x2(2, 5);
  x1 << 0,   0,  0, .8, .8,
        0, -.5, .8,  0, .8;
  x2 << 0,    0,  0, .8, .8,
        .1, -.4, .9,  .1, .9; // Y Translated camera.
  typedef essential::FivePointKernel Kernel;
  Kernel kernel(x1,x2, Mat3::Identity(), Mat3::Identity());

  bool is_ok = true;
  std::vector<size_t> samples;
  for (size_t i = 0; i < x1.cols(); ++i) {
    samples.push_back(i);
  }
  std::vector<Mat3> Es;
  kernel.Fit(samples, &Es);

  is_ok &= (!Es.empty());
  for (int i = 0; i < Es.size(); ++i) {
    for(int j = 0; j < x1.cols(); ++j)
      EXPECT_NEAR(0.0, kernel.Error(j,Es[i]), 1e-8);
  }
}

TEST(FivePointKernelTest, FivePointsRelativePose_Kernel) {

  typedef essential::FivePointKernel Kernel;

  int focal = 1000;
  int principal_Point = 500;

  //-- Setup a circular camera rig and assert that 5PT relative pose works.
  const int iNviews = 8;
  NViewDataSet d = NRealisticCamerasRing(iNviews, Kernel::MINIMUM_SAMPLES,
    NViewDatasetConfigurator(focal,focal,principal_Point,principal_Point,5,0)); // Suppose a camera with Unit matrix as K

  size_t found = 0;
  for(int i=1; i <iNviews; ++i)
  {
	  std::vector<Mat3> Es, Rs;  // Essential, Rotation matrix.
	  std::vector<Vec3> ts;      // Translation matrix.

    // Direct value do not work.
    // As we use reference, it cannot convert Mat2X& to Mat&
    Mat x0 = d.projected_points_[0];
    Mat x1 = d.projected_points_[i];

    Kernel kernel(x0, x1, d.camera_matrix_[0], d.camera_matrix_[1]);
	std::vector<size_t> samples;
    for (size_t k = 0; k < Kernel::MINIMUM_SAMPLES; ++k) {
      samples.push_back(k);
    }
    kernel.Fit(samples, &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d.projected_points_[0].col(0);
      x2Col << d.projected_points_[i].col(0);
	  EXPECT_TRUE(
        MotionFromEssentialAndCorrespondence(Es[s],
        d.camera_matrix_[0], x1Col,
        d.camera_matrix_[i], x2Col,
        &Rs[s],
        &ts[s]));
    }
    //-- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d.rotation_matrix_[0], d.translation_vector_[0], d.rotation_matrix_[i], d.translation_vector_[i], &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {

      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-4);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    //-- Almost one solution must find the correct relative orientation
	EXPECT_TRUE(bsolution_found);
    if (bsolution_found)
      found++;
  }
  EXPECT_EQ(iNviews - 1, found);
}
