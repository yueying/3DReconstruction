
#include "testing.h"
#include "fblib/utils/notify.h"
#include "fblib/multiview/projection.h"
#include "fblib/multiview/homography.h"

using namespace fblib::multiview;
using namespace fblib::utils;

// Check whether homography transform M actually transforms
// given vectors x1 to x2. Used to check validness of a reconstructed
// homography matrix.
// TODO(sergey): Consider using this in all tests since possible homography
// matrix is not fixed to a single value and different-looking matrices
// might actually crrespond to the same exact transform.
void CheckHomography2DTransform(const Mat3 &H,
                                const Mat &x1,
                                const Mat &x2) {
  for (int i = 0; i < x2.cols(); ++i) {
    Vec3 x2_expected = x2.col(i);
    Vec3 x2_observed = H * x1.col(i);
    x2_observed /= x2_observed(2);
    EXPECT_MATRIX_NEAR(x2_expected, x2_observed, 1e-8);
  }
}


TEST(Homography2DTest, Rotation45AndTranslationXY) {
  Mat x1(3, 4);
  x1 <<  0, 1, 0, 5,
         0, 0, 2, 3,
         1, 1, 1, 1;

  double angle = 45.0;
  Mat3 m;
  m << cos(angle), -sin(angle), -2,
       sin(angle),  cos(angle),  5,
       0,           0,           1;

  Mat x2 = x1;
  // Transform point from ground truth matrix
  for (int i = 0; i < x2.cols(); ++i)
    x2.col(i) = m * x1.col(i);

  Mat3 homography_mat;
  EXPECT_TRUE(Homography2DFromCorrespondencesLinear(x1, x2, &homography_mat));
  notify(INFO) << "Mat Homography2D ";
  notify(INFO) << homography_mat;
  notify(INFO) << "Mat GT ";
  notify(INFO) << m;
  EXPECT_MATRIX_NEAR(homography_mat, m, 1e-8);
}

TEST(Homography2DTest, AffineGeneral4) {
  // TODO(julien) find why it doesn't work with 4 points!!!
  Mat x1(3, 4);
  x1 << 0, 1, 0, 2,
        0, 0, 1, 2,
        1, 1, 1, 1;
  Mat3 m;
  m << 3, -1,  4,
       6, -2, -3,
       0,  0,  1;

  Mat x2 = x1;
  for (int i = 0; i < x2.cols(); ++i) {
    x2.col(i) = m * x1.col(i);
  }

  Mat3 homography_mat;
  EXPECT_TRUE(Homography2DFromCorrespondencesLinear(x1, x2, &homography_mat));
  notify(INFO) << "Mat Homography2D";
  notify(INFO) << homography_mat;
  CheckHomography2DTransform(homography_mat, x1, x2);

  // Test with euclidean coordinates
  Mat eX1, eX2;
  HomogeneousToEuclidean(x1, &eX1);
  HomogeneousToEuclidean(x2, &eX2);
  homography_mat.setIdentity();
  EXPECT_TRUE(Homography2DFromCorrespondencesLinear(eX1, eX2, &homography_mat));

  notify(INFO) << "Mat Homography2D ";
  notify(INFO) << homography_mat;
  CheckHomography2DTransform(homography_mat, x1, x2);
}

TEST(Homography2DTest, AffineGeneral5) {
  Mat x1(3, 5);
  x1 <<  0, 1, 0, 2, 5,
         0, 0, 1, 2, 2,
         1, 1, 1, 1, 1;
  Mat3 m;
  m <<   3, -1,  4,
         6, -2, -3,
         0,  0,  1;

  Mat x2 = x1;
  for (int i = 0; i < x2.cols(); ++i)
    x2.col(i) = m * x1.col(i);

  Mat3 homography_mat;
  EXPECT_TRUE(Homography2DFromCorrespondencesLinear(x1, x2, &homography_mat));

  notify(INFO) << "Mat Homography2D ";
  notify(INFO) << homography_mat;
  EXPECT_MATRIX_NEAR(homography_mat, m, 1e-8);

  // Test with euclidean coordinates
  Mat eX1, eX2;
  HomogeneousToEuclidean(x1, &eX1);
  HomogeneousToEuclidean(x2, &eX2);
  homography_mat.setIdentity();
  EXPECT_TRUE(Homography2DFromCorrespondencesLinear(eX1, eX2, &homography_mat));

  notify(INFO) << "Mat Homography2D ";
  notify(INFO) << homography_mat;
  EXPECT_MATRIX_NEAR(homography_mat, m, 1e-8);
}

TEST(Homography2DTest, HomographyGeneral) {
  Mat x1(3, 4);
  x1 <<  0, 1, 0, 5,
         0, 0, 2, 3,
         1, 1, 1, 1;
  Mat3 m;
  m <<   3, -1,  4,
         6, -2, -3,
         1, -3,  1;

  Mat x2 = x1;
  for (int i = 0; i < x2.cols(); ++i)
    x2.col(i) = m * x1.col(i);

  Mat3 homography_mat;
  EXPECT_TRUE(Homography2DFromCorrespondencesLinear(x1, x2, &homography_mat));

  notify(INFO) << "Mat Homography2D ";
  notify(INFO) << homography_mat;
  EXPECT_MATRIX_NEAR(homography_mat, m, 1e-8);
}

TEST(Homography3DTest, RotationAndTranslationXYZ) {
  Mat x1(4, 5);
  x1 <<  0, 0, 1, 5, 2,
         0, 1, 2, 3, 5,
         0, 2, 0, 1, 5,
         1, 1, 1, 1, 1;
  Mat4 M;
  M.setIdentity();
  /*
  M = AngleAxisd(45.0, Vector3f::UnitZ())
    * AngleAxisd(25.0, Vector3f::UnitX())
    * AngleAxisd(5.0, Vector3f::UnitZ());*/

  // Rotation on x + translation
  double angle = 45.0;
  Mat4 rot;
  rot <<  1,          0,           0,  1,
          0, cos(angle), -sin(angle),  3,
          0, sin(angle),  cos(angle), -2,
          0,          0,           0,  1;
  M *= rot;
  // Rotation on y
  angle = 25.0;
  rot <<  cos(angle), 0, sin(angle),  0,
          0,          1,          0,  0,
         -sin(angle), 0, cos(angle),  0,
          0,           0,          0, 1;
  M *= rot;
  // Rotation on z
  angle = 5.0;
  rot <<  cos(angle), -sin(angle), 0, 0,
          sin(angle),  cos(angle), 0, 0,
          0,           0,          1, 0,
          0,           0,          0, 1;
  M *= rot;
  Mat x2 = x1;
  for (int i = 0; i < x2.cols(); ++i) {
    x2.col(i) = M * x1.col(i);
  }

  Mat4 homography_mat;
  EXPECT_TRUE(Homography3DFromCorrespondencesLinear(x1, x2, &homography_mat));

  notify(INFO) << "Mat Homography3D " << homography_mat;
  notify(INFO) << "Mat GT " << M;
  EXPECT_MATRIX_NEAR(homography_mat, M, 1e-8);
}

TEST(Homography3DTest, AffineGeneral) {
  Mat x1(4, 5);
  x1 <<  0, 0, 1, 5, 2,
         0, 1, 2, 3, 5,
         0, 2, 0, 1, 5,
         1, 1, 1, 1, 1;
  Mat4 m;
  m <<   3, -1,  4,  1,
         6, -2, -3, -6,
         1,  0,  1,  2,
         0,  0,  0,  1;

  Mat x2 = x1;
  for (int i = 0; i < x2.cols(); ++i) {
    x2.col(i) = m * x1.col(i);
  }

  Mat4 homography_mat;
  EXPECT_TRUE(Homography3DFromCorrespondencesLinear(x1, x2, &homography_mat));
  notify(INFO) << "Mat Homography3D ";
  notify(INFO) << homography_mat;
  EXPECT_MATRIX_NEAR(homography_mat, m, 1e-8);
}

TEST(Homography3DTest, HomographyGeneral) {
  Mat x1(4, 5);
  x1 << 0, 0, 1, 5, 2,
        0, 1, 2, 3, 5,
        0, 2, 0, 1, 5,
        1, 1, 1, 1, 1;
  Mat4 m;
  m <<  3, -1,  4,  1,
        6, -2, -3, -6,
        1,  0,  1,  2,
       -3,  1,  0,  1;

  Mat x2 = x1;
  for (int i = 0; i < x2.cols(); ++i) {
    x2.col(i) = m * x1.col(i);
  }

  Mat4 homography_mat;
  EXPECT_TRUE(Homography3DFromCorrespondencesLinear(x1, x2, &homography_mat));
  notify(INFO) << "Mat Homography3D";
  notify(INFO) << homography_mat;
  EXPECT_MATRIX_NEAR(homography_mat, m, 1e-8);
}

