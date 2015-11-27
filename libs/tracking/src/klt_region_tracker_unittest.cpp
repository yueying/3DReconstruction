#include "mvg/tracking/klt_region_tracker.h"
#include "mvg/image/image.h"
#include "testing.h"

using namespace mvg::image;
using namespace mvg::tracking;

TEST(KltRegionTracker, Track) {
  Image<float> image1(51, 51);
  image1.fill(0);

  Image<float> image2(image1);

  int x0 = 25, y0 = 25;
  int dx = 3, dy = 2;
  image1(y0, x0) = 1.0f;
  image2(y0 + dy, x0 + dx) = 1.0f;

  double x1 = x0;
  double y1 = y0;

  KltRegionTracker tracker;
  tracker.half_window_size = 6;
  EXPECT_TRUE(tracker.Track(image1, image2, x0, y0, &x1, &y1));

  EXPECT_NEAR(x1, x0 + dx, 0.001);
  EXPECT_NEAR(y1, y0 + dy, 0.001);
}

