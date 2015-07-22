#ifndef FBLIB_TRACKING_TRKLT_REGION_TRACKER_H_
#define FBLIB_TRACKING_TRKLT_REGION_TRACKER_H_

#include "fblib/image/image.h"
#include "fblib/tracking/region_tracker.h"
using namespace fblib::image;
namespace fblib {
	namespace tracking{
		// An improved KLT algorithm that enforces that the tracking is time-reversible
		// [1]. This is not the same as the "symmetric" KLT that is sometimes used.
		// Anecdotally, this tracks much more consistently than vanilla KLT.
		//
		// [1] H. Wu, R. Chellappa, and A. Sankaranarayanan and S. Kevin Zhou. Robust
		//     visual tracking using the time-reversibility constraint. International
		//     Conference on Computer Vision (ICCV), Rio de Janeiro, October 2007.
		//
		struct TrkltRegionTracker : public RegionTracker {
			TrkltRegionTracker()
				: half_window_size(4),
				max_iterations(100),
				min_determinant(1e-6),
				min_update_squared_distance(1e-6),
				sigma(0.9),
				lambda(0.05) {}

			virtual ~TrkltRegionTracker() {}

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
			double lambda;
		};
	}
}  // namespace fblib

#endif  // FBLIB_TRACKING_TRKLT_REGION_TRACKER_H_
