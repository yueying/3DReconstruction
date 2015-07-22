#ifndef FBLIB_CORRESPONDENCE_PYRAMID_TRACKER_H_
#define FBLIB_CORRESPONDENCE_PYRAMID_TRACKER_H_

#include "fblib/image/image.h"
#include "fblib/utils/scoped_ptr.h"
#include "fblib/tracking/region_tracker.h"
using namespace fblib::image;
using namespace fblib::utils;
namespace fblib {
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
}  // namespace fblib

#endif  // FBLIB_CORRESPONDENCE_PYRAMID_TRACKER_H_
