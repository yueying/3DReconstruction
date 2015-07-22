#ifndef FBLIB_IMAGE_IMAGE_WARPING_H_
#define FBLIB_IMAGE_IMAGE_WARPING_H_

#include "fblib/math/numeric.h"
using namespace fblib::math;

namespace fblib{
	namespace image{

		/// Apply inplace homography transform for the given point (x,y).
		/// Return true if homography_matrix is orientation preserving around the point.
		bool ApplyH_AndCheckOrientation(const Mat3 &homography_matrix, double &x, double &y)
		{
			Vec3 X(x, y, 1.0);
			X = homography_matrix*X;
			X /= X(2);
			x = X(0);
			y = X(1);
			return (X(2) * homography_matrix(2, 2) > 0.0);
		}

		/// Warp an image input_image given a homography homography_matrix with a backward approach
		/// homography_matrix must be already have been resized accordingly
		template <class Image>
		void Warp(const Image &input_image, const Mat3 &homography_matrix, Image &output_image)
		{
			const int out_image_width = static_cast<int>(output_image.Width());
			const int out_image_height = static_cast<int>(output_image.Height());

			for (int j = 0; j < out_image_height; ++j)
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
				for (int i = 0; i < out_image_width; ++i)
				{
				double xT = i, yT = j;
				if (ApplyH_AndCheckOrientation(homography_matrix, xT, yT)
					&& input_image.Contains(yT, xT))
					output_image(j, i) = SampleLinear(input_image, (float)yT, (float)xT);
				}
		}

	}; // namespace image
}; // namespace fblib


#endif // FBLIB_IMAGE_IMAGE_WARPING_H_
