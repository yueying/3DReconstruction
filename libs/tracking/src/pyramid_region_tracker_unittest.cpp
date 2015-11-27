#include "mvg/tracking/pyramid_region_tracker.h"
#include "mvg/tracking/klt_region_tracker.h"
#include "mvg/image/image.h"
#include "testing.h"

using namespace mvg::image;
using namespace mvg::tracking;

TEST(PyramidKltRegionTracker, Track) {
  Image<float> image1(100, 100);
  image1.fill(0);

  Image<float> image2(image1);

  int x1 = 25, y1 = 25;
  image1(y1 + 0, x1 + 0) = 1.0f;
  image1(y1 + 0, x1 + 1) = 1.0f;
  image1(y1 + 1, x1 + 0) = 1.0f;
  image1(y1 + 1, x1 + 1) = 1.0f;

  // Make the displacement too large for a single-level KLT.
  int x2 = x1 + 6, y2 = y1 + 5;
  image2(y2 + 0, x2 + 0) = 1.0f;
  image2(y2 + 0, x2 + 1) = 1.0f;
  image2(y2 + 1, x2 + 0) = 1.0f;
  image2(y2 + 1, x2 + 1) = 1.0f;

  // Use a small 5x5 tracking region.
  int half_window_size = 3;

  // Ensure that the track doesn't work with one level of KLT.
  {
    double x2_actual = x1;
    double y2_actual = y1;

    KltRegionTracker tracker;
    tracker.half_window_size = half_window_size;
    EXPECT_FALSE(tracker.Track(image1, image2, x1, y1,
                               &x2_actual, &y2_actual));
  }

  // Verify that it works with the pyramid tracker.
  {
    double x2_actual = x1;
    double y2_actual = y1;

    KltRegionTracker *klt_tracker = new KltRegionTracker;
    klt_tracker->half_window_size = half_window_size;

    PyramidRegionTracker tracker(klt_tracker, 3);
    EXPECT_TRUE(tracker.Track(image1, image2, x1, y1,
                              &x2_actual, &y2_actual));

    EXPECT_NEAR(x2_actual, x2, 0.001);
    EXPECT_NEAR(y2_actual, y2, 0.001);
  }
}

