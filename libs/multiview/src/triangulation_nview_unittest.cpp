#include "mvg/multiview/triangulation_nview.h"
#include "mvg/multiview/nview_data_sets.h"
#include "testing.h"

using namespace mvg::multiview;

TEST(Triangulate_NView, FiveViews) {
  int nviews = 5;
  int npoints = 6;
  NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  // Collect P matrices together.
  std::vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j) {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i) {
    // Collect the image of point i in each frame.
    Mat2X xs(2, nviews);
    for (int j = 0; j < nviews; ++j) {
      xs.col(j) = d.projected_points_[j].col(i);
    }
    Vec4 X;
    TriangulateNView(xs, Ps, &X);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j) {
      Vec3 x_reprojected = Ps[j]*X;
      x_reprojected /= x_reprojected(2);
      double error = (x_reprojected.head(2) - xs.col(j)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}

TEST(Triangulate_NViewAlgebraic, FiveViews) {
  int nviews = 5;
  int npoints = 6;
  NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  // Collect P matrices together.
  std::vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j) {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i) {
    // Collect the image of point i in each frame.
    Mat2X xs(2, nviews);
    for (int j = 0; j < nviews; ++j) {
      xs.col(j) = d.projected_points_[j].col(i);
    }
    Vec4 X;
    TriangulateNViewAlgebraic(xs, Ps, &X);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j) {
      Vec3 x_reprojected = Ps[j]*X;
      x_reprojected /= x_reprojected(2);
      double error = (x_reprojected.head<2>() - xs.col(j)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}

TEST(Triangulate_NViewIterative, FiveViews) {
  int nviews = 5;
  int npoints = 6;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  // Collect P matrices together.
  std::vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j) {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i) {

    Triangulation triangulationObj;
    for (int j = 0; j < nviews; ++j)
    triangulationObj.add(Ps[j], d.projected_points_[j].col(i));

    Vec3 X = triangulationObj.compute();
    // Check reprojection error. Should be nearly zero.
    EXPECT_NEAR(triangulationObj.error(X), 0.0, 1e-9);
    for (int j = 0; j < nviews; ++j) {
      Vec3 x_reprojected = Ps[j]* Vec4(X(0), X(1), X(2), 1.0);
      x_reprojected /= x_reprojected(2);
      double error = (x_reprojected.head<2>() - d.projected_points_[j].col(i)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}

