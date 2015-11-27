#ifndef MVG_IMAGE_PIXEL_TYPES_H_
#define MVG_IMAGE_PIXEL_TYPES_H_

#include "mvg/math/numeric.h"
#include "mvg/image/traits.h"

namespace mvg{
	namespace image{

		/**
		 * \brief	像素类型RGB模板
		 *
		 * \tparam	T	rgb存放像素值的类型（uchar，float）
		 */
		template <typename T>
		class Rgb : public Eigen::Matrix < T, 3, 1, 0, 3, 1 >
		{			
			typedef Eigen::Matrix<T, 3, 1, 0, 3, 1> Base;
			typedef T TBase;

		public:
			/** \brief	构造函数 */
			inline Rgb() : Base(0, 0, 0) {}
			inline Rgb(T red, T green, T blue) : Base(red, green, blue){}
			explicit inline Rgb(const Base& val) : Base(val) {}
			explicit inline Rgb(const T t[3]) : Base(t) {}
			explicit inline Rgb(const T val) : Base(val, val, val) {}
			
			/**	得到对应的rgb值
			 */
			inline const T& r() const { return (*this)(0); }
			inline T& r() { return (*this)(0); }
			inline const T& g() const { return (*this)(1); }
			inline T& g() { return (*this)(1); }
			inline const T& b() const { return (*this)(2); }
			inline T& b() { return (*this)(2); }
			/**	得到对应的灰度值
			 */
			//inline operator T() const { return T(0.3*r() + *0.59*g() + 0.11*b()); }
			

			friend std::ostream& operator<<(std::ostream& os, const Rgb& col)
			{
				os << " {";
				for (int i = 0; i < 2; ++i)
					os << col(i) << ",";
				os << col(2) << "} ";
				return os;
			}

			template <typename Z>
			inline Rgb operator /(const Z& val) const
			{
				return Rgb(T((Z)((*this)(0)) / val),
					T((Z)((*this)(1)) / val),
					T((Z)((*this)(2)) / val));
			}

			template <typename Z>
			inline Rgb operator *(const Z& val) const
			{
				return Rgb(T((Z)(*this)(0) * val),
					T((Z)(*this)(1) * val),
					T((Z)(*this)(2) * val));
			}

			

		};
		typedef Rgb<unsigned char> RGBColor;
		typedef Rgb<float> RGBfColor;

		/**
		* \brief	像素类型RGBA模板
		*
		* \tparam	T	rgba存放像素值的类型（uchar，float）
		*/
		template <typename T>
		class Rgba : public Eigen::Matrix < T, 4, 1, 0, 4, 1 >
		{
			typedef Eigen::Matrix<T, 4, 1, 0, 4, 1> Base;

		public:

			inline Rgba() : Base(0, 0, 0, 0) {}
			inline Rgba(T red, T green, T blue, T alpha = 0)
				: Base(red, green, blue, alpha){}
			explicit inline Rgba(const Base& val) : Base(val) {}
			explicit inline Rgba(const T t[4]) : Base(t) {}
			explicit inline Rgba(const T val) : Base(val, val, val, 1.0) {}
			inline Rgba(const RGBColor val)
				: Base(val.r(), val.g(), val.b(), static_cast<T>(1)) {}

			inline const T& r() const { return (*this)(0); }
			inline T& r() { return (*this)(0); }
			inline const T& g() const { return (*this)(1); }
			inline T& g() { return (*this)(1); }
			inline const T& b() const { return (*this)(2); }
			inline T& b() { return (*this)(2); }
			inline const T& a() const { return (*this)(3); }
			inline T& a() { return (*this)(3); }
			/**	得到灰度图像，不考虑alpha通道
			 */
			inline operator T() const { return T(0.3*r() + *0.59*g() + 0.11*b()); }

			friend std::ostream& operator<<(std::ostream& os, const Rgba& col) {
				os << " {";
				for (int i = 0; i < 3; ++i)
					os << col(i) << ",";
				os << col(3) << "} ";
				return os;
			}

			template<class Z>
			inline Rgba operator /(const Z& val) const
			{
				return Rgba(T((Z)(*this)(0) / val),
					T((Z)(*this)(1) / val),
					T((Z)(*this)(2) / val),
					T((Z)(*this)(3) / val));
			}

			template<class Z>
			inline Rgba operator *(const Z& val) const
			{
				return Rgba(T((Z)(*this)(0) * val),
					T((Z)(*this)(1) * val),
					T((Z)(*this)(2) * val),
					T((Z)(*this)(3) * val));
			}

			
		};
		typedef Rgba<unsigned char> RGBAColor;

		const RGBColor WHITE(255, 255, 255);
		const RGBColor BLACK(0, 0, 0);
		const RGBColor BLUE(0, 0, 255);
		const RGBColor RED(255, 0, 0);
		const RGBColor GREEN(0, 255, 0);
		const RGBColor YELLOW(255, 255, 0);
		const RGBColor CYAN(0, 255, 255);
		const RGBColor MAGENTA(255, 0, 255);

		template<> class DataType < RGBColor >
		{
		public:
			typedef RGBColor        value_type;
			typedef unsigned char  channel_type;
			enum {
				depth = IMAGE_DEPTH_8U,
				channels = 3
			};
		};

		template<> class DataType < RGBfColor >
		{
		public:
			typedef RGBfColor        value_type;
			typedef float  channel_type;
			enum {
				depth = IMAGE_DEPTH_32F,
				channels = 3
			};
		};
			template<> class DataType < RGBAColor >
			{
			public:
				typedef RGBAColor        value_type;
				typedef unsigned char  channel_type;
				enum {
					depth = IMAGE_DEPTH_8U,
					channels = 4
				};
			};
			
	}
}

#endif // MVG_IMAGE_PIXEL_TYPES_H_

