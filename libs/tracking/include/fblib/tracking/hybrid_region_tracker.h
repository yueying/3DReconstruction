#ifndef FBLIB_TRACKING_HYBRID_REGION_TRACKER_H_
#define FBLIB_TRACKING_HYBRID_REGION_TRACKER_H_

#include "fblib/image/image.h"
#include "fblib/utils/scoped_ptr.h"
#include "fblib/tracking/region_tracker.h"
using namespace fblib::image;
using namespace fblib::utils;
namespace fblib {
	namespace tracking{
		// TODO(keir): Documentation!
		class HybridRegionTracker : public RegionTracker {
		public:
			HybridRegionTracker(RegionTracker *coarse_tracker,
				RegionTracker *fine_tracker)
				: coarse_tracker_(coarse_tracker),
				fine_tracker_(fine_tracker) {}

			virtual ~HybridRegionTracker() {}

			// Tracker interface.
			virtual bool Track(const Image<float> &image1,
				const Image<float> &image2,
				double  x1, double  y1,
				double *x2, double *y2) const;

			scoped_ptr<RegionTracker> coarse_tracker_;
			scoped_ptr<RegionTracker> fine_tracker_;
		};
	}
}  // namespace fblib

#endif  // FBLIB_TRACKING_HYBRID_REGION_TRACKER_H_
