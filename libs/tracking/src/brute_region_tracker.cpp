#include "tracking_precomp.h"
#include "mvg/tracking/brute_region_tracker.h"

#include "mvg/utils/aligned_malloc.h"
#include "mvg/image/image.h"
#include "mvg/image/correlation.h"
#include "mvg/image/sample.h"
#include "mvg/utils/notify.h"

using namespace mvg::image;
using namespace mvg::utils;

namespace mvg {
	namespace tracking{

		/**	检查区域是否在图像内
		 */
		bool RegionIsInBounds(const Image<float> &image1,
			double x, double y,
			int half_window_size) {
			// 检查最小的坐标
			int min_x = floor(x) - half_window_size - 1;
			int min_y = floor(y) - half_window_size - 1;
			if (min_x < 0.0 ||
				min_y < 0.0) {
				return false;
			}

			// 检查最大的坐标
			int max_x = ceil(x) + half_window_size + 1;
			int max_y = ceil(y) + half_window_size + 1;
			if (max_x > image1.cols() ||
				max_y > image1.rows()) {
				return false;
			}

			return true;
		}

		// Computes the sum of absolute differences between pattern and image. Pattern
		// must be 16-byte aligned, and the stride must be a multiple of 16. The image
		// does pointer does not have to be aligned.
		int SumOfAbsoluteDifferencesContiguousImage(
			const unsigned char *pattern,
			unsigned int pattern_width,
			unsigned int pattern_height,
			unsigned int pattern_stride,
			const unsigned char *image,
			unsigned int image_stride) {

			int sad = 0;
			for (int r = 0; r < pattern_height; ++r) {
				for (int c = 0; c < pattern_width; ++c) {
					sad += abs(pattern[pattern_stride * r + c] - image[image_stride * r + c]);
				}
			}
			return sad;

		}

		// Sample a region of size width, height centered at x,y in image, converting
		// from float to byte in the process. Samples from the first channel. Puts
		// result into *pattern.
		void SampleRectangularPattern(const Image<RGBfColor> &image,
			double x, double y,
			int width,
			int height,
			int pattern_stride,
			unsigned char *pattern) {
			// There are two cases for width and height: even or odd. If it's odd, then
			// the bounds [-width / 2, width / 2] works as expected. However, for even,
			// this results in one extra access past the end. So use < instead of <= in
			// the loops below, but increase the end limit by one in the odd case.
			int end_width = (width / 2) + (width % 2);
			int end_height = (height / 2) + (height % 2);
			for (int r = -height / 2; r < end_height; ++r) {
				for (int c = -width / 2; c < end_width; ++c) {
					pattern[pattern_stride * (r + height / 2) + c + width / 2] =
						(SampleLinear(image, y + r, x + c))(0) * 255.0;
				}
			}
		}

		// Returns x rounded up to the nearest multiple of alignment.
		inline int PadToAlignment(int x, int alignment) {
			if (x % alignment != 0) {
				x += alignment - (x % alignment);
			}
			return x;
		}

		// Sample a region centered at x,y in image with size extending by half_width
		// from x. Samples from the first channel. The resulting array is placed in
		// *pattern, and the stride, which will be a multiple of 16 if SSE is enabled,
		// is returned in *pattern_stride.
		//
		// NOTE: Caller must free *pattern with aligned_malloc() from above.
		void SampleSquarePattern(const Image<RGBfColor> &image,
			double x, double y,
			int half_width,
			unsigned char **pattern,
			int *pattern_stride) {
			int width = 2 * half_width + 1;
			// Allocate an aligned block with padding on the end so each row of the
			// pattern starts on a 16-byte boundary.
			*pattern_stride = PadToAlignment(width, 16);
			int pattern_size_bytes = *pattern_stride * width;
			*pattern = static_cast<unsigned char *>(
				aligned_malloc(pattern_size_bytes, 16));
			SampleRectangularPattern(image, x, y, width, width,
				*pattern_stride,
				*pattern);
		}

		// NOTE: Caller must free *image with aligned_malloc() from above.
		void FloatArrayToByteArrayWithPadding(const Image<RGBfColor> &float_image,
			unsigned char **image,
			int *image_stride) {
			// Allocate enough so that accessing 16 elements past the end is fine.
			*image_stride = float_image.Width() + 16;
			*image = static_cast<unsigned char *>(
				aligned_malloc(*image_stride * float_image.Height(), 16));
			for (int i = 0; i < float_image.Height(); ++i) {
				for (int j = 0; j < float_image.Width(); ++j) {
					(*image)[*image_stride * i + j] =
						static_cast<unsigned char>(255.0 * float_image(i, j)(0));
				}
			}
		}


		// TODO(keir): Compare the "sharpness" of the peak around the best pixel. It's
		// probably worth plotting a few examples to see what the histogram of SAD
		// values for every hypothesis looks like.
		//
		// TODO(keir): Priority queue for multiple hypothesis.
		bool BruteRegionTracker::Track(const Image<float> &image1,
			const Image<float> &image2,
			double  x1, double  y1,
			double *x2, double *y2) const {
			if (!RegionIsInBounds(image1, x1, y1, half_window_size)) {
				notify(INFO) << "Fell out of image1's window with x1=" << x1 << ", y1=" << y1
					<< ", hw=" << half_window_size << ".";
				return false;
			}
			int pattern_width = 2 * half_window_size + 1;

			Image<RGBfColor> image_and_gradient1;
			Image<RGBfColor> image_and_gradient2;
			BlurredImageAndDerivativesChannels(image1, 0.9, &image_and_gradient1);
			BlurredImageAndDerivativesChannels(image2, 0.9, &image_and_gradient2);

			// Sample the pattern to get it aligned to an image grid.
			unsigned char *pattern;
			int pattern_stride;
			SampleSquarePattern(image_and_gradient1, x1, y1, half_window_size,
				&pattern,
				&pattern_stride);

			// Convert the search area directly to bytes without sampling.
			unsigned char *search_area;
			int search_area_stride;
			FloatArrayToByteArrayWithPadding(image_and_gradient2, &search_area,
				&search_area_stride);

			// Try all possible locations inside the search area. Yes, everywhere.
			int best_i = -1, best_j = -1, best_sad = INT_MAX;
			for (int i = 0; i < image2.Height() - pattern_width; ++i) {
				for (int j = 0; j < image2.Width() - pattern_width; ++j) {
					int sad = SumOfAbsoluteDifferencesContiguousImage(pattern,
						pattern_width,
						pattern_width,
						pattern_stride,
						search_area + search_area_stride * i + j,
						search_area_stride);
					if (sad < best_sad) {
						best_i = i;
						best_j = j;
						best_sad = sad;
					}
				}
			}

			//CHECK_NE(best_i, -1);
			//CHECK_NE(best_j, -1);

			aligned_free(pattern);
			aligned_free(search_area);

			if (best_sad == INT_MAX) {
				notify(INFO) << "Hit INT_MAX in SAD; failing.";
				return false;
			}

			*x2 = best_j + half_window_size;
			*y2 = best_i + half_window_size;

			// Calculate the shift done by the fine tracker.
			double dx2 = *x2 - x1;
			double dy2 = *y2 - y1;
			double fine_shift = sqrt(dx2 * dx2 + dy2 * dy2);
			notify(INFO) << "Brute shift: dx=" << dx2 << " dy=" << dy2 << ", d=" << fine_shift;

			if (minimum_correlation <= 0) {
				// No correlation checking requested; nothing else to do.
				notify(INFO) << "No correlation checking; returning success. best_sad: " << best_sad;
				return true;
			}

			Image<RGBfColor> image_and_gradient1_sampled, image_and_gradient2_sampled;
			SamplePattern(image_and_gradient1, x1, y1, half_window_size, 
				&image_and_gradient1_sampled);
			SamplePattern(image_and_gradient2, *x2, *y2, half_window_size, 
				&image_and_gradient2_sampled);

			// Compute the Pearson product-moment correlation coefficient to check
			// for sanity.
			double correlation = PearsonProductMomentCorrelation(
				image_and_gradient1_sampled,
				image_and_gradient2_sampled);

			notify(INFO) << "Final correlation: " << correlation;

			if (correlation < minimum_correlation) {
				notify(INFO) << "Correlation " << correlation << " greater than "
					<< minimum_correlation << "; bailing.";
				return false;
			}
			return true;
		}
	} //namespace tracking
}  // namespace mvg
