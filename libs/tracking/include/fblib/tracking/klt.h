#ifndef FBLIB_TRACKING_KLT_H_
#define FBLIB_TRACKING_KLT_H_

#include "fblib/image/image.h"
using namespace fblib::image;

namespace fblib {
	namespace tracking{

		class Tracker {
		public:
			Tracker() {}
			/*!
				Construct a tracker to track the pattern centered in \a image1.

				The tracked pattern is a \a half_pattern_size * 2 + 1 patch in the center of image1
				\a image1 should be a square patch of size \a search_size
				This tracker will use pyramid tracking using \a num_levels levels.
				*/
			Tracker(const Image<float> &image1, float x, float y, int half_pattern_size,
				int search_width, int search_height, int num_levels);
			/*!
				Track a point from last image to \a image2.

				\a x2, \a y2 should start out as a best guess for the position in \a
				image2. If no guess is available, (\a x1, \a y1) is a good start. Returns
				true on success, false otherwise

				\a image2 become the "last image" of this tracker.
				*/
			bool Track(const Image<float> &image2, float *x2, float *y2);

		private:
			void MakePyramid(const Image<float> &image, float** pyramid) const;
			bool TrackImage(const float* image1,
				const float* image2,
				int size,
				int half_pattern_size,
				float x1, float y1, float *x2, float *y2) const;

			int half_pattern_size;
			int search_width;
			int search_height;
			int num_levels;
			int max_iterations;
			float tolerance;
			float min_determinant;
			float min_update_squared_distance;
			float sigma;
			float lambda;
			float* pyramid1[8];
			float x1, y1;
		};
	} //namespace tracking
}  // namespace fblib

#endif  // FBLIB_TRACKING_KLT_H_
