#ifndef FBLIB_TRACKING_RETRACK_REGION_TRACKER_H_
#define FBLIB_TRACKING_RETRACK_REGION_TRACKER_H_

#include "fblib/image/image.h"
#include "fblib/utils/scoped_ptr.h"
#include "fblib/tracking/region_tracker.h"
using namespace fblib::image;
using namespace fblib::utils;
namespace fblib {
	namespace tracking{
		// A region tracker that tries tracking backwards and forwards, rejecting a
		// track that doesn't track backwards to the starting point.
		class RetrackRegionTracker : public RegionTracker {
		public:
			RetrackRegionTracker(RegionTracker *tracker, double tolerance)
				: tracker_(tracker), tolerance_(tolerance) {}

			virtual bool Track(const Image<float> &image1,
				const Image<float> &image2,
				double  x1, double  y1,
				double *x2, double *y2) const;
		private:
			scoped_ptr<RegionTracker> tracker_;
			double tolerance_;
		};
	}
}  // namespace fblib

#endif  // FBLIB_TRACKING_RETRACK_REGION_TRACKER_H_
