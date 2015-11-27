#ifndef MVG_TRACKING_RETRACK_REGION_TRACKER_H_
#define MVG_TRACKING_RETRACK_REGION_TRACKER_H_

#include "mvg/image/image.h"
#include "mvg/utils/scoped_ptr.h"
#include "mvg/tracking/region_tracker.h"
using namespace mvg::image;
using namespace mvg::utils;
namespace mvg {
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
}  // namespace mvg

#endif  // MVG_TRACKING_RETRACK_REGION_TRACKER_H_
