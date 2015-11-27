#ifndef MVG_CORRESPONDENCE_PYRAMID_TRACKER_H_
#define MVG_CORRESPONDENCE_PYRAMID_TRACKER_H_

#include "mvg/image/image.h"
#include "mvg/utils/scoped_ptr.h"
#include "mvg/tracking/region_tracker.h"
using namespace mvg::image;
using namespace mvg::utils;
namespace mvg {
	namespace tracking{
		class PyramidRegionTracker : public RegionTracker {
		public:
			PyramidRegionTracker(RegionTracker *tracker, int num_levels)
				: tracker_(tracker), num_levels_(num_levels) {}

			virtual bool Track(const Image<float> &image1,
				const Image<float> &image2,
				double  x1, double  y1,
				double *x2, double *y2) const;
		private:
			scoped_ptr<RegionTracker> tracker_;
			int num_levels_;
		};
	}
}  // namespace mvg

#endif  // MVG_CORRESPONDENCE_PYRAMID_TRACKER_H_
