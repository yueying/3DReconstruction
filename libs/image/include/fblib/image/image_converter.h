#ifndef FBLIB_IMAGE_IMAGE_CONVERTER_H_
#define FBLIB_IMAGE_IMAGE_CONVERTER_H_

#include "fblib/image/image_container.h"
#include "fblib/image/pixel_types.h"

namespace fblib{
	namespace image{

		/**
		 * \brief	RGB转灰度 ，权重值参考： http://www.easyrgb.com/
		 *          R * 0.2126 + G * 0.7152 + B * 0.0722
		 */
		template<typename T>		
		inline T Rgb2Gray(const T r, const T g, const T b) {
			return r * 0.2126 + g * 0.7152 + b * 0.0722;
		}

		/**	图像间的转换
		 */
		template<typename Tin, typename Tout>
		inline void Convert(const Tin &val_in, Tout &out) {
			out = static_cast<Tout>(val_in);
		}

		template<>
		inline void Convert<RGBColor, unsigned char>(
			const RGBColor &val_in, unsigned char &val_out)
		{
			val_out = static_cast<unsigned char>(0.3 * val_in.r() + 0.59 * val_in.g() + 0.11 * val_in.b());
		}

		template<>
		inline void Convert<RGBAColor, unsigned char>(
			const RGBAColor &val_in, unsigned char &val_out)
		{
			val_out = static_cast<unsigned char>(
				(val_in.a() / 255.f) *
				(0.3 * val_in.r() + 0.59 * val_in.g() + 0.11 * val_in.b()));
		}

		template<>
		inline void Convert<RGBAColor, RGBColor>(
			const RGBAColor &val_in, RGBColor &val_out)
		{
			val_out = RGBColor(
				static_cast<unsigned char> ((val_in.a() / 255.f) * val_in.r()),
				static_cast<unsigned char> ((val_in.a() / 255.f) * val_in.g()),
				static_cast<unsigned char> ((val_in.a() / 255.f) * val_in.b()));
		}

		template<typename ImageIn, typename ImageOut>
		void ConvertPixelType(const ImageIn &img_in, ImageOut *img_out)
		{
			(*img_out) = ImageOut(img_in.Width(), img_in.Height());
			// 对每个像素值进行处理
			for (int j = 0; j < img_in.Height(); ++j)
				for (int i = 0; i < img_in.Width(); ++i)
					Convert(img_in(j, i), (*img_out)(j, i));
		}

		/**	将RGB ( unsigned char or int ) 转 Float
		 */
		template< typename Tin, typename Tout >
		inline void ConvertRGB2Float(
			const Tin &var_in,
			Tout &val_out,
			float factor = 1.0f / 255.f)
		{
			for (int channel = 0; channel < 3; ++channel)
				val_out(channel) = (float)((int)(var_in(channel)) * factor);
		}

		template< typename ImageIn >
		void Rgb2Float(const ImageIn& img_in,
			Image< RGBfColor > *img_out, float factor = 1.0f / 255.f)
		{
			assert(img_in.Channels() == 3);
			(*img_out).resize(img_in.Width(), img_in.Height());
			// 将每个 int RGB 转 float RGB 值
			for (int j = 0; j < img_in.Height(); ++j)
				for (int i = 0; i < img_in.Width(); ++i)
					ConvertRGB2Float(img_in(j, i), (*img_out)(j, i), factor);
		}
				
		/**	将Float转RGB( unsigned char or int )
		 */
		static void ConvertFloatToInt(
			const RGBfColor& var_in,
			RGBColor& val_out,
			float factor = 255.f)
		{
			for (int channel = 0; channel < 3; ++channel)
				val_out(channel) = (int)(var_in(channel) * factor);
		}

		static void RgbFloat2RgbInt(
			const Image< RGBfColor >& img_in,
			Image< RGBColor > *img_out,
			float factor = 255.f)
		{
			assert(img_in.Channels() == 3);
			(*img_out).resize(img_in.Width(), img_in.Height());
			for (int j = 0; j < img_in.Height(); ++j)
				for (int i = 0; i < img_in.Width(); ++i)
					ConvertFloatToInt(img_in(j, i), (*img_out)(j, i), factor);
		}

		/**	将三个单通道存储在多通道中，注意，这边不仅仅是rgb，可能是计算单通道图像平滑，图像x，y两方向的梯度
		 */
		static void ThreeChannels2RGBf(const Image<float> &channel1,const Image<float> &channel2,
			const Image<float> &channel3, Image< RGBfColor > *img_out)
		{
			// TODO(fengbing) 这边可能再考虑一下，怎么安排
			(*img_out).resize(channel1.Width(), channel1.Height());
			// 将每个 int RGB 转 float RGB 值
			for (int j = 0; j < channel1.Height(); ++j)
				for (int i = 0; i < channel1.Width(); ++i)
				{
				(*img_out)(j, i)(0) = (float)((int)(channel1(j, i)));
				(*img_out)(j, i)(1) = (float)((int)(channel2(j, i)));
				(*img_out)(j, i)(2) = (float)((int)(channel3(j, i)));
				}
		}


	}// namespace image
} // namespace fblib

#endif // FBLIB_IMAGE_IMAGE_CONVERTER_H_
