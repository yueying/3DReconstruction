#include "fblib/multiview/projection.h"
#include "fblib/multiview/nview_data_sets.h"
#include "fblib/multiview/triangulation.h"
#include "testing.h"

using namespace fblib::multiview;
using namespace std;

TEST(Triangulation, TriangulateDLT) {

  NViewDataSet d = NRealisticCamerasRing(2, 12);

  for (int i = 0; i < d.point_3d_.cols(); ++i) {
    Vec2 x1, x2;
    x1 = d.projected_points_[0].col(i);
    x2 = d.projected_points_[1].col(i);
    Vec3 X_estimated, X_gt;
    X_gt = d.point_3d_.col(i);
    TriangulateDLT(d.P(0), x1, d.P(1), x2, &X_estimated);
    EXPECT_NEAR(0, DistanceLInfinity(X_estimated, X_gt), 1e-8);
  }
}

