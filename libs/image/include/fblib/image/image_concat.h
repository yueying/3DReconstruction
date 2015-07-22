#ifndef FBLIB_IMAGE_IMAGE_CONCAT_H_
#define FBLIB_IMAGE_IMAGE_CONCAT_H_

#include "fblib/image/image_container.h"

namespace fblib{
	namespace image{

		/**
		 * \brief	将图像进行横向连接
		 *
		 * \tparam	Image	图像类型
		 * \param	left_image	   	输入左边的图像
		 * \param	right_image	   	输入右边的图像
		 * \param [in,out]	out_image	输出的图像
		 */
		template < class Image >
		void ConcatHorizontal(const Image &left_image, const Image &right_image, Image &out_image)
		{
			// 计算新的图像维度 // |left_image|+|right_image|
			int left_add_right_width = left_image.Width() + right_image.Width();
			out_image.resize(left_add_right_width, std::max(left_image.Height(), right_image.Height()));

			// 复制左边的图像 |left_image|...|
			out_image.block(0, 0, left_image.Height(), left_image.Width()) = left_image.GetMat();
			// 再复制右边的图像 |left_image|right_image|
			out_image.block(0, left_image.Width(), right_image.Height(), right_image.Width()) = right_image.GetMat();
		}

		/**
		* \brief	将图像进行竖向连接
		*
		* \tparam	Image	图像类型
		* \param	up_image	   	输入上边的图像
		* \param	up_image	   	输入下边的图像
		* \param [in,out]	out_image	输出的图像
		*/
		template < class Image >
		void ConcatVertical(const Image &up_image, const Image &down_image, Image &out_image)
		{
			// 计算新的图像维度
			// |up_image|
			// |down_image|
			int hh = up_image.Height() + down_image.Height();
			out_image.resize(max(up_image.Width(), down_image.Width()), hh);

			// 复制上边的图像
			// |up_image|
			// |....|
			out_image.block(0, 0, up_image.Height(), up_image.Width()) = up_image.GetMat();
			// 复制下边的图像
			// |up_image|
			// |down_image|
			out_image.block(up_image.Height(), 0, down_image.Height(), down_image.Width()) = down_image.GetMat();
		}

	}// namespace image
}// namespace fblib

#endif // FBLIB_IMAGE_IMAGE_CONCAT_H_
