#include "tracking_precomp.h"
#include "mvg/tracking/retrack_region_tracker.h"

#include <cmath>
#include <vector>
using namespace mvg::image;
using namespace mvg::utils;
namespace mvg {
	namespace tracking{
		bool RetrackRegionTracker::Track(const Image<float> &image1,
			const Image<float> &image2,
			double  x1, double  y1,
			double *x2, double *y2) const {
			// Track forward, getting x2 and y2.
			if (!tracker_->Track(image1, image2, x1, y1, x2, y2)) {
				return false;
			}
			// Now track x2 and y2 backward, to get xx1 and yy1 which, if the track is
			// good, should match x1 and y1 (but may not if the track is bad).
			double xx1 = *x2, yy1 = *x2;
			if (!tracker_->Track(image2, image1, *x2, *y2, &xx1, &yy1)) {
				return false;
			}
			double dx = xx1 - x1;
			double dy = yy1 - y1;
			return sqrt(dx * dx + dy * dy) < tolerance_;
		}
	}
}  // namespace mvg
