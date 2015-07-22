#include "tracking_precomp.h"
#include "fblib/tracking/pyramid_region_tracker.h"

#include <vector>

#include "fblib/image/convolve.h"
#include "fblib/image/image.h"
#include "fblib/image/sample.h"
#include "fblib/utils/notify.h"
using namespace fblib::image;
using namespace fblib::utils;
namespace fblib {
	namespace tracking{
		static void MakePyramid(const Image<float> &image, int num_levels,
			std::vector<Image<float>> *pyramid) {
			pyramid->resize(num_levels);
			(*pyramid)[0] = image;
			for (int i = 1; i < num_levels; ++i) {
				DownsampleChannelsBy2((*pyramid)[i - 1], &(*pyramid)[i]);
			}
		}

		bool PyramidRegionTracker::Track(const Image<float> &image1,
			const Image<float> &image2,
			double  x1, double  y1,
			double *x2, double *y2) const {
			// Shrink the guessed x and y location to match the coarsest level + 1 (which
			// when gets corrected in the loop).
			*x2 /= pow(2., num_levels_);
			*y2 /= pow(2., num_levels_);

			// Create all the levels of the pyramid, since tracking has to happen from
			// the coarsest to finest levels, which means holding on to all levels of the
			// pyraid at once.
			std::vector<Image<float>> pyramid1(num_levels_);
			std::vector<Image<float>> pyramid2(num_levels_);
			MakePyramid(image1, num_levels_, &pyramid1);
			MakePyramid(image2, num_levels_, &pyramid2);

			for (int i = num_levels_ - 1; i >= 0; --i) {
				// Position in the first image at pyramid level i.
				double xx = x1 / pow(2., i);
				double yy = y1 / pow(2., i);

				// Guess the new tracked position is where the last level tracked to.
				*x2 *= 2;
				*y2 *= 2;

				// Save the previous best guess for this level, since tracking within this
				// level might fail.
				double x2_new = *x2;
				double y2_new = *y2;

				// Track the point on this level with the base tracker.
				notify(INFO) << "Tracking on level " << i;
				bool succeeded = tracker_->Track(pyramid1[i], pyramid2[i], xx, yy,
					&x2_new, &y2_new);

				if (!succeeded) {
					if (i == 0) {
						// Only fail on the highest-resolution level, because a failure on a
						// coarse level does not mean failure at a lower level (consider
						// out-of-bounds conditions).
						notify(INFO) << "Finest level of pyramid tracking failed; failing.";
						return false;
					}

					notify(INFO) << "Failed to track at level " << i << "; restoring guess.";
				}
				else {
					// Only save the update if the track for this level succeeded. This is a
					// bit of a hack; the jury remains out on whether this is better than
					// re-using the previous failed-attempt.
					*x2 = x2_new;
					*y2 = y2_new;
				}
			}
			return true;
		}
	}
}  // namespace fblib
