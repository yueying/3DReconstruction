#include "mvg/tracking/hybrid_region_tracker.h"

#include "mvg/image/image.h"
#include "mvg/image/convolve.h"
#include "mvg/image/sample.h"
#include "mvg/utils/notify.h"

using namespace mvg::image;
using namespace mvg::utils;

namespace mvg {
	namespace tracking{
		bool HybridRegionTracker::Track(const Image<float> &image1,
			const Image<float> &image2,
			double  x1, double  y1,
			double *x2, double *y2) const {
			double x2_coarse = *x2;
			double y2_coarse = *y2;
			if (!coarse_tracker_->Track(image1, image2, x1, y1, &x2_coarse, &y2_coarse)) {
				notify(INFO) << "Coarse tracker failed.";
				return false;
			}

			double x2_fine = x2_coarse;
			double y2_fine = y2_coarse;
			if (!fine_tracker_->Track(image1, image2, x1, y1, &x2_fine, &y2_fine)) {
				notify(INFO) << "Fine tracker failed.";
				return false;
			}

			// Calculate the shift done by the fine tracker.
			double dx2 = x2_coarse - x2_fine;
			double dy2 = y2_coarse - y2_fine;
			double fine_shift = sqrt(dx2 * dx2 + dy2 * dy2);

			notify(INFO) << "Refinement: dx=" << dx2 << " dy=" << dy2 << ", d=" << fine_shift;

			// If the fine tracker shifted the window by more than a pixel, then
			// something bad probably happened and we should give up tracking.
			if (fine_shift < 2.0) {
				notify(INFO) << "Refinement small enough; success.";
				*x2 = x2_fine;
				*y2 = y2_fine;
				return true;
			}
			notify(INFO) << "Refinement was too big; failing.";
			return false;
		}
	}// namespace tracking
}  // namespace mvg
