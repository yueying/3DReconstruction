#ifndef FBLIB_IMAGE_IMAGE_CONTAINER_H_
#define FBLIB_IMAGE_IMAGE_CONTAINER_H_

#include "fblib/math/numeric.h"
#include "fblib/image/traits.h"

/**	基于Eigen 构建通过的图像处理算法
 */
namespace fblib{
	namespace image{
		/**	图像容器，保存图像的宽度和高度，存储数据，数据存储以行优先，数据访问可以通过operator(y,x)
		 */
		template <typename T>
		class Image : public Eigen::Matrix < T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor >
		{

		public:
			typedef T Tpixel;	//!<像素类型
			typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Base;//!< 图像基类矩阵

			/**	构造函数
			 */
			inline Image() {};

			inline Image(int width, int height, bool is_init = true, const T val = T())
			{
				Base::resize(height, width);
				if (is_init) Base::fill(val);
			};

			inline Image(const Base& image)  { (*this) = image; }

			virtual inline ~Image() {};

			inline Image& operator=(const Base& image) { Base::operator=(image);  return *this; }
			
			
			inline void Resize(int width, int height, bool is_init = true, const T val = T())
			{
				Base::resize(height, width);
				if (is_init) Base::fill(val);
			}

			template<typename D>
			void ResizeLike(const Image<D> &other) {
				Resize(other.Width(),other.Height());
			}

			/**
			 * \brief	得到图像的宽度
			 *
			 * \return	宽度值
			 */
			inline int Width()  const { return static_cast<int>(Base::cols()); }

			/**
			 * \brief	得到图像的高度
			 *
			 * \return	高度值
			 */
			inline int Height() const { return static_cast<int>(Base::rows()); }

			/**
			 * \brief	返回图像像素的深度信息
			 *
			 * \return	深度值 （unsigned char类型返回 1）
			 */
			inline int Depth() const  { return DataType<Tpixel>::depth; }

			/**
			* \brief	返回图像像素的通道数
			*
			* \return	通道数 （unsigned char类型返回 1）
			*/
			inline int Channels() const  { return DataType<Tpixel>::channels; }

			/**
			 * \brief	返回步长信息
			 *
			 * \return	步长
			 */
			inline int Stride() const { return this->stride(); }

			/**
			 * \brief	返回size大小
			 *
			 * \return	返回步长*类型宽度
			 */
			inline int Size() const { return this->stride()*sizeof(Tpixel); }

			/**
			 * \brief	重载操作，访问(y,x)下像素值
			 *
			 * \param	y	所在行号
			 * \param	x	所在列号
			 *
			 * \return	返回(y,x)下像素值
			 */
			inline const T& operator()(int y, int x) const { return Base::operator()(y, x); }
			inline T& operator()(int y, int x) { return Base::operator()(y, x); }

			/**
			 * \brief	返回图像矩阵
			 *
			 * \return	图像矩阵
			 */
			inline const Base& GetMat() const { return (*this); }

			/**
			 * \brief	测试给定点是否在图像内
			 *
			 * \param	y	图像y方向坐标
			 * \param	x	图像x方向坐标
			 *
			 * \return	true 表示在图像内, false 表示不在图像内.
			 */
			inline bool Contains(int y, int x) const  {
				return 0 <= x && x < Base::cols()
					&& 0 <= y && y < Base::rows();
			}

		protected:
			//--通过继承一个矩阵存储图像数据
		};

	}// namespace image 
}// namespace fblib

#endif // FBLIB_IMAGE_IMAGE_CONTAINER_H_