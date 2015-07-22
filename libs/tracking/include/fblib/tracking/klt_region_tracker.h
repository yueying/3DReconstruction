#ifndef FBLIB_TRACKING_KLT_REGION_TRACKER_H_
#define FBLIB_TRACKING_KLT_REGION_TRACKER_H_

#include "fblib/image/image.h"
#include "fblib/tracking/region_tracker.h"
using namespace fblib::image;
namespace fblib {
	namespace tracking{
		struct KltRegionTracker : public RegionTracker {
			KltRegionTracker()
				: half_window_size(4),
				max_iterations(16),
				min_determinant(1e-6),
				min_update_squared_distance(1e-6),
				sigma(0.9) {}

			virtual ~KltRegionTracker() {}

			// Tracker interface.
			virtual bool Track(const Image<float> &image1,
				const Image<float> &image2,
				double  x1, double  y1,
				double *x2, double *y2) const;

			// No point in creating getters or setters.
			int half_window_size;
			int max_iterations;
			double min_determinant;
			double min_update_squared_distance;
			double sigma;
		};
	}
}  // namespace fblib

#endif  // FBLIB_TRACKING_KLT_REGION_TRACKER_H_
