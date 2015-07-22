#ifndef FBLIB_IMAGE_SAMPLE_H_
#define FBLIB_IMAGE_SAMPLE_H_

#include "fblib/image/image_container.h"
#include "fblib/image/pixel_types.h"

namespace fblib {
	namespace image{

		/**
		 * \brief	采用最近邻插值
		 *
		 * \tparam	T	图像类型
		 * \param	image	输入图像
		 * \param	y	 	y轴坐标
		 * \param	x	 	x轴坐标
		 *
		 * \return	返回（y,x)坐标
		 */
		template<typename T>
		inline T SampleNearest(const Image<T>& image,
			float y, float x) {
			return image(static_cast<int>(round(y)), static_cast<int>(round(x)));
		}

		/**	线性插值初始化，确定当前点离周边四点的权重
		 */
		inline void LinearInitAxis(float x, int size,
			int *x1, int *x2,
			float *dx) {
			const int ix = static_cast<int>(x);
			if (ix < 0) {
				*x1 = 0;
				*x2 = 0;
				*dx = 1.0;
			}
			else if (ix > size - 2) {
				*x1 = size - 1;
				*x2 = size - 1;
				*dx = 1.0;
			}
			else {
				*x1 = ix;
				*x2 = ix + 1;
				*dx = *x2 - x;
			}
		}

		/**
		 * \brief	线性插值
		 */
		template<typename T>
		inline T SampleLinear(const Image<T>& image, float y, float x) {
			int x1, y1, x2, y2;
			float dx, dy;

			LinearInitAxis(y, image.Height(), &y1, &y2, &dy);
			LinearInitAxis(x, image.Width(), &x1, &x2, &dx);

			const T im11 = image(y1, x1);
			const T im12 = image(y1, x2);
			const T im21 = image(y2, x1);
			const T im22 = image(y2, x2);

			return T(dy  * (dx * im11 + (1.0 - dx) * im12) +
				(1.0 - dy) * (dx * im21 + (1.0 - dx) * im22));
		}

		/**	线性插值，RGB特例
		 */
		template<>
		inline RGBColor SampleLinear<RGBColor>(const Image<RGBColor>& image, float y, float x) {
			int x1, y1, x2, y2;
			float dx, dy;

			LinearInitAxis(y, image.Height(), &y1, &y2, &dy);
			LinearInitAxis(x, image.Width(), &x1, &x2, &dx);

			const Rgb<float> im11((image(y1, x1).cast<float>()));
			const Rgb<float> im12((image(y1, x2).cast<float>()));
			const Rgb<float> im21((image(y2, x1).cast<float>()));
			const Rgb<float> im22((image(y2, x2).cast<float>()));

			return RGBColor(
				(
				(im11 * dx + im12 * (1 - dx)) * dy +
				(im21 * dx + im22 * (1 - dx)) * (1 - dy)
				)
				.cast<unsigned char>()
				);
		}
		template<typename T>
		inline void SampleLinear(const Image<T> &image, float y, float x, T *sample) {
			int x1, y1, x2, y2;
			float dx, dy;

			LinearInitAxis(y, image.Height(), &y1, &y2, &dy);
			LinearInitAxis(x, image.Width(), &x1, &x2, &dx);


			const T im11 = image(y1, x1);
			const T im12 = image(y1, x2);
			const T im21 = image(y2, x1);
			const T im22 = image(y2, x2);

			*sample = T(dy  * (dx * im11 + (1.0 - dx) * im12) +
					(1 - dy) * (dx * im21 + (1.0 - dx) * im22));
			
		}

		/**	2x2盒子滤波的降采样
		 */
		template<typename T>
		inline void DownsampleChannelsBy2(const Image<T> &in, Image<T> *out) {
			int height = in.Height() / 2;
			int width = in.Width() / 2;
			out->Resize(height, width);
			for (int r = 0; r < height; ++r) {
				for (int c = 0; c < width; ++c) {
					(*out)(r, c) = (in(2 * r, 2 * c) +
						in(2 * r + 1, 2 * c) +
						in(2 * r, 2 * c + 1) +
						in(2 * r + 1, 2 * c + 1)) / 4.0f;
				}
			}

		}
		/**	给出RGBfColor特例
		 */
		template<>
		inline void DownsampleChannelsBy2(const Image<RGBfColor> &in, Image<RGBfColor> *out) {
			int height = in.Height() / 2;
			int width = in.Width() / 2;
			int channels = in.Channels();
			out->Resize(height, width);
			for (int r = 0; r < height; ++r) {
				for (int c = 0; c < width; ++c) {
					for (int m = 0; m < channels; ++m)
						(*out)(r, c)[m] = (in(2 * r, 2 * c)(m)+
						in(2 * r + 1, 2 * c)(m)+
						in(2 * r, 2 * c + 1)(m)+
						in(2 * r + 1, 2 * c + 1)(m)) / 4.0f;
				}
			}

		}

		/**	对一个区域进行降采样
		 */
		template<typename T>
		inline void SamplePattern(const Image<T> &image,
			double x, double y,
			int half_width,
			Image<T> *sampled) {
			sampled->Resize(2 * half_width + 1, 2 * half_width + 1);
			for (int r = -half_width; r <= half_width; ++r) {
				for (int c = -half_width; c <= half_width; ++c) {
					(*sampled)(r + half_width, c + half_width) =
						SampleLinear(image, y + r, x + c);
				}
			}
		}
	}
}  // namespace fblib



#endif // FBLIB_IMAGE_SAMPLE_H_
