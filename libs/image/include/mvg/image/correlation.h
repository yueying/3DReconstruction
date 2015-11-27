
#ifndef MVG_IMAGE_CORRELATION_H
#define MVG_IMAGE_CORRELATION_H

#include "mvg/utils/notify.h"
#include "mvg/image/image.h"

namespace mvg {
	namespace image{

		inline double PearsonProductMomentCorrelation(
			const Image<RGBfColor> &image_and_gradient1_sampled,
			const Image<RGBfColor> &image_and_gradient2_sampled) {
			assert(image_and_gradient1_sampled.Width() ==
				image_and_gradient2_sampled.Width());
			assert(image_and_gradient1_sampled.Height() ==
				image_and_gradient2_sampled.Height());

			const int width = image_and_gradient1_sampled.Width(),
				height = image_and_gradient1_sampled.Height();
			double sX = 0, sY = 0, sXX = 0, sYY = 0, sXY = 0;

			for (int r = 0; r < height; ++r) {
				for (int c = 0; c < width; ++c) {
					double x = image_and_gradient1_sampled(r, c)(0);
					double y = image_and_gradient2_sampled(r, c)(0);
					sX += x;
					sY += y;
					sXX += x * x;
					sYY += y * y;
					sXY += x * y;
				}
			}

			// Normalize.
			double N = width * height;
			sX /= N;
			sY /= N;
			sXX /= N;
			sYY /= N;
			sXY /= N;

			double var_x = sXX - sX * sX;
			double var_y = sYY - sY * sY;
			double covariance_xy = sXY - sX * sY;

			double correlation = covariance_xy / sqrt(var_x * var_y);
			mvg::utils::notify(mvg::utils::INFO) << "Covariance xy: " << covariance_xy
				<< ", var 1: " << var_x << ", var 2: " << var_y
				<< ", correlation: " << correlation;
			return correlation;
		}
	}// namespace image
}  // namespace mvg

#endif  // MVG_IMAGE_IMAGE_CORRELATION_H
