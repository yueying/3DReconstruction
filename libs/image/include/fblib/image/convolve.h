#ifndef FBLIB_IMAGE_CONVOLVE_H_
#define FBLIB_IMAGE_CONVOLVE_H_

#include "fblib/image/image.h"
#include "fblib/math/numeric.h"

using namespace fblib::math;

// TODO(fengbing) : 找到一个好地方存放如下函数，比如： gaussian.h in math ?
namespace fblib {
	namespace image{

		/**
		 * \brief	一维零均值高斯函数
		 *
		 * \param	x	 	内核大小
		 * \param	sigma	标准偏差
		 *
		 * \return	高斯函数值
		 */		
		inline double Gaussian(double x, double sigma) {
			return 1 / sqrt(2 * M_PI*sigma*sigma) * exp(-(x*x / 2 / sigma / sigma));
		}
		
		/**	二维零均值高斯函数
		 *  参考： http://mathworld.wolfram.com/GaussianFunction.html
		 */
		inline double Gaussian2D(double x, double y, double sigma) {
			return 1.0 / (2.0*M_PI*sigma*sigma) * exp(-(x*x + y*y) / (2.0*sigma*sigma));
		}

		/**	求解高斯函数的导数
		 */
		inline double GaussianDerivative(double x, double sigma) {
			return -x / sigma / sigma * Gaussian(x, sigma);
		}
		
		/**	根据高斯函数值反求解内核大小
		 */
		inline double GaussianInversePositive(double y, double sigma) {
			return sqrt(-2 * sigma * sigma * log(y * sigma * sqrt(2 * M_PI)));
		}

		/**	计算高斯内核及其导数
		 */
		void ComputeGaussianKernel(double sigma, Vec *kernel, Vec *derivative);

		/**	水平卷积
		 */
		void ConvolveHorizontal(const Image<float> &in,
			const Vec &kernel,
			Image<float> *out_pointer,
			int plane = -1);
		/**	垂直卷积
		 */
		void ConvolveVertical(const Image<float> &in,
			const Vec &kernel,
			Image<float> *out_pointer,
			int plane = -1);
		/**	高斯卷积
		 */
		void ConvolveGaussian(const Image<float> &in,
			double sigma,
			Image<float> *out_pointer);

		/**	计算图像梯度
		 */
		void ImageDerivatives(const Image<float> &in,
			double sigma,
			Image<float> *gradient_x,
			Image<float> *gradient_y);

		/**	对图像进行平滑，并给出x、y方向图像梯度
		 */
		void BlurredImageAndDerivatives(const Image<float> &in,
			double sigma,
			Image<float> *blurred_image,
			Image<float> *gradient_x,
			Image<float> *gradient_y);

		/**	对图像进行平滑和计算梯度保存到三通道图像中
		 */
		void BlurredImageAndDerivativesChannels(const Image<float> &in,
			double sigma,
			Image<RGBfColor> *blurred_and_gradxy);

		/**	水平盒子滤波
		 */
		void BoxFilterHorizontal(const Image<float> &in,
			int window_size,
			Image<float> *out_pointer);
		/**	垂直盒子滤波
		 */
		void BoxFilterVertical(const Image<float> &in,
			int window_size,
			Image<float> *out_pointer);
		/**	盒子滤波
		 */
		void BoxFilter(const Image<float> &in,
			int box_width,
			Image<float> *out);

		/**	拉普拉斯平滑
		 */
		void LaplaceFilter(unsigned char* src,
			unsigned char* dst,
			int width,
			int height,
			int strength);

	} // namespace image
}  // namespace fblib

#endif  // FBLIB_IMAGE_CONVOLVE_H_

