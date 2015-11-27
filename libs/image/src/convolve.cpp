#include "mvg/image/convolve.h"
#include <cmath>
#include "mvg/image/image.h"
#include "mvg/math/numeric.h"

using namespace mvg::math;

namespace mvg {
	namespace image{
		
		void ComputeGaussianKernel(double sigma, Vec *kernel, Vec *derivative) {
			
			assert(sigma >= 0.0);
			// sigma=1,kernel=3的高斯值
			const float truncation_factor = 0.004f;
			// 基于sigma求解内核大小
			float precisehalfwidth = GaussianInversePositive(truncation_factor, sigma);
			int width = lround(2 * precisehalfwidth);
			if (width % 2 == 0) {
				width++;
			}
			// 计算高斯内核及其导数
			kernel->resize(width);
			derivative->resize(width);
			kernel->setZero();
			derivative->setZero();
			int halfwidth = width / 2;
			for (int i = -halfwidth; i <= halfwidth; ++i)  {
				(*kernel)(i + halfwidth) = Gaussian(i, sigma);
				(*derivative)(i + halfwidth) = GaussianDerivative(i, sigma);
			}
			//进行归一化处理
			NormalizeL1(kernel);

			// www.cs.duke.edu/courses/spring03/cps296.1/handouts/Image%20Processing.pdf
			double factor = 0.;
			for (int i = -halfwidth; i <= halfwidth; ++i)  {
				factor -= i*(*derivative)(i + halfwidth);
			}
			*derivative /= factor;
		}

		/**	快速卷积操作
		 */
		template <int size, bool vertical>
		void FastConvolve(const Vec &kernel, int width, int height,
			const float* src, int src_stride, int src_line_stride,
			float* dst, int dst_stride) {

			double coefficients[2 * size + 1];
			for (int k = 0; k < 2 * size + 1; ++k) {
				coefficients[k] = kernel(2 * size - k);
			}
			// 内核大小固定，循环处理
			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					double sum = 0;
					for (int k = -size; k <= size; ++k) {
						if (vertical) {
							if (y + k >= 0 && y + k < height) {
								sum += src[k * src_line_stride] * coefficients[k + size];
							}
						}
						else {
							if (x + k >= 0 && x + k < width) {
								sum += src[k * src_stride] * coefficients[k + size];
							}
						}
					}
					dst[0] = static_cast<float>(sum);
					src += src_stride;
					dst += dst_stride;
				}
			}
		}

		template<bool vertical>
		void Convolve(const Image<float> &in,const Vec &kernel,
			Image<float> *out_pointer,int plane) {
			int width = in.Width();
			int height = in.Height();
			Image<float> &out = *out_pointer;
			if (plane == -1) {
				out.ResizeLike(in);
				plane = 0;
			}

			assert(kernel.size() % 2 == 1);
			assert(&in != out_pointer);

			int src_line_stride = in.rowStride();
			int src_stride = in.colStride();
			int dst_stride = out.colStride();
			const float* src = in.GetMat().data();
			float t = src[0];
			float* dst = const_cast<float*>(out.GetMat().data()) + plane;

			// 应对不同内核大小
			int half_width = kernel.size() / 2;
			switch (half_width) {

#define static_convolution(size) case size: \
  FastConvolve<size, vertical>(kernel, width, height, src, src_stride, \
                               src_line_stride, dst, dst_stride); break;
				static_convolution(1)
					static_convolution(2)
					static_convolution(3)
					static_convolution(4)
					static_convolution(5)
					static_convolution(6)
					static_convolution(7)
#undef static_convolution

default:
	int dynamic_size = kernel.size() / 2;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			double sum = 0;
			// 卷积处理
			for (int k = -dynamic_size; k <= dynamic_size; ++k) {
				if (vertical) {
					if (y + k >= 0 && y + k < height) {
						sum += src[k * src_line_stride] *
							kernel(2 * dynamic_size - (k + dynamic_size));
					}
				}
				else {
					if (x + k >= 0 && x + k < width) {
						sum += src[k * src_stride] *
							kernel(2 * dynamic_size - (k + dynamic_size));
					}
				}
			}
			dst[0] = static_cast<float>(sum);
			src += src_stride;
			dst += dst_stride;
		}
	}
			}
		}

		void ConvolveHorizontal(const Image<float> &in,
			const Vec &kernel,
			Image<float> *out_pointer,
			int plane) {
			Convolve<false>(in, kernel, out_pointer, plane);
		}

		void ConvolveVertical(const Image<float> &in,
			const Vec &kernel,
			Image<float> *out_pointer,
			int plane) {
			Convolve<true>(in, kernel, out_pointer, plane);
		}

		void ConvolveGaussian(const Image<float> &in,
			double sigma,
			Image<float> *out_pointer) {
			Vec kernel, derivative;
			ComputeGaussianKernel(sigma, &kernel, &derivative);

			Image<float> tmp;
			ConvolveVertical(in, kernel, &tmp);
			ConvolveHorizontal(tmp, kernel, out_pointer);
		}

		void ImageDerivatives(const Image<float> &in,
			double sigma,
			Image<float> *gradient_x,
			Image<float> *gradient_y) {
			Vec kernel, derivative;
			ComputeGaussianKernel(sigma, &kernel, &derivative);
			Image<float> tmp;

			// 计算x方向的梯度
			ConvolveVertical(in, kernel, &tmp);
			ConvolveHorizontal(tmp, derivative, gradient_x);

			// 计算y方向的梯度
			ConvolveHorizontal(in, kernel, &tmp);
			ConvolveVertical(tmp, derivative, gradient_y);
		}

		void BlurredImageAndDerivatives(const Image<float> &in,
			double sigma,
			Image<float> *blurred_image,
			Image<float> *gradient_x,
			Image<float> *gradient_y) {
			Vec kernel, derivative;
			ComputeGaussianKernel(sigma, &kernel, &derivative);
			Image<float> tmp;

			// 计算图像卷积
			ConvolveVertical(in, kernel, &tmp);
			ConvolveHorizontal(tmp, kernel, blurred_image);

			// 计算x方向导数
			ConvolveHorizontal(tmp, derivative, gradient_x);

			// 计算y方向导数
			ConvolveHorizontal(in, kernel, &tmp);
			ConvolveVertical(tmp, derivative, gradient_y);
		}

		/**	计算三通道图像高斯模糊及梯度
		 */
		void BlurredImageAndDerivativesChannels(const Image<float> &in,
			double sigma,
			Image<RGBfColor> *blurred_and_gradxy) {

			assert(in.Channels() == 1);

			Vec kernel, derivative;
			ComputeGaussianKernel(sigma, &kernel, &derivative);

			// 计算卷积图像
			Image<float> tmp;
			ConvolveVertical(in, kernel, &tmp);
			blurred_and_gradxy->Resize(in.Height(), in.Width());
			Image<float> blurred_and_gradxy_chan1, blurred_and_gradxy_chan2, blurred_and_gradxy_chan3;
			ConvolveHorizontal(tmp, kernel, &blurred_and_gradxy_chan1);

			// 计算x方向导数
			ConvolveHorizontal(tmp, derivative, &blurred_and_gradxy_chan2);

			// 计算y方向导数
			ConvolveHorizontal(in, kernel, &tmp);
			ConvolveVertical(tmp, derivative, &blurred_and_gradxy_chan3);

			ThreeChannels2RGBf(blurred_and_gradxy_chan1, blurred_and_gradxy_chan2, blurred_and_gradxy_chan3, blurred_and_gradxy);
		}

		void BoxFilterHorizontal(const Image<float> &in,
			int window_size,
			Image<float> *out_pointer) {
			Image<float> &out = *out_pointer;
			out.ResizeLike(in);
			int half_width = (window_size - 1) / 2;

			for (int i = 0; i < in.Height(); ++i) {
				float sum = 0;
				// 初始化
				for (int j = 0; j < half_width; ++j) {
					sum += in(i, j);
				}
				// 计算左边界
				for (int j = 0; j < half_width + 1; ++j) {
					sum += in(i, j + half_width);
					out(i, j) = sum;
				}
				// 计算中间部分
				for (int j = half_width + 1; j < in.Width() - half_width; ++j) {
					sum -= in(i, j - half_width - 1);
					sum += in(i, j + half_width);
					out(i, j) = sum;
				}
				// 计算右边界
				for (int j = in.Width() - half_width; j < in.Width(); ++j) {
					sum -= in(i, j - half_width - 1);
					out(i, j) = sum;
				}
			}

		}

		void BoxFilterVertical(const Image<float> &in,
			int window_size,
			Image<float> *out_pointer) {
			Image<float> &out = *out_pointer;
			out.ResizeLike(in);
			int half_width = (window_size - 1) / 2;

			for (int j = 0; j < in.Width(); ++j) {
				float sum = 0;
				// 初始化
				for (int i = 0; i < half_width; ++i) {
					sum += in(i, j);
				}
				// 计算上边界
				for (int i = 0; i < half_width + 1; ++i) {
					sum += in(i + half_width, j);
					out(i, j) = sum;
				}
				// 计算中间部分
				for (int i = half_width + 1; i < in.Height() - half_width; ++i) {
					sum -= in(i - half_width - 1, j);
					sum += in(i + half_width, j);
					out(i, j) = sum;
				}
				// 计算下边界
				for (int i = in.Height() - half_width; i < in.Height(); ++i) {
					sum -= in(i - half_width - 1, j);
					out(i, j) = sum;
				}
			}

		}

		void BoxFilter(const Image<float> &in,
			int box_width,
			Image<float> *out) {
			Image<float> tmp;
			BoxFilterHorizontal(in, box_width, &tmp);
			BoxFilterVertical(tmp, box_width, out);
		}

		void LaplaceFilter(unsigned char* src,
			unsigned char* dst,
			int width,
			int height,
			int strength) {
			for (int y = 1; y < height - 1; y++)
				for (int x = 1; x < width - 1; x++) {
				const unsigned char* s = &src[y*width + x];
				int l = 128 +
					s[-width - 1] + s[-width] + s[-width + 1] +
					s[1] - 8 * s[0] + s[1] +
					s[width - 1] + s[width] + s[width + 1];
				int d = ((256 - strength)*s[0] + strength*l) / 256;
				if (d < 0) d = 0;
				if (d > 255) d = 255;
				dst[y*width + x] = d;
				}
		}

	} //namespace image
}  // namespace mvg
