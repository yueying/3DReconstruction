/*******************************************************************************
 * 文件： degree_radian_convert.h
 * 时间： 2014/12/03 17:14
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 角度与弧度转换
 *
********************************************************************************/
#ifndef FBLIB_MATH_ANGLE_RADIAN_CONVERT_H_
#define FBLIB_MATH_ANGLE_RADIAN_CONVERT_H_

#include <cmath>
#include <fblib/utils/fblib_macros.h>

namespace fblib
{
	namespace math
	{
#ifdef DEG2RAD  // 这边确保函数优于宏
#undef DEG2RAD
#endif
#ifdef RAD2DEG
#undef RAD2DEG
#endif
#if !defined(M_PI)
#	define M_PI 3.14159265358979323846
#endif

		/** 度转弧度 */
		inline double DEG2RAD(const double x) { return x*M_PI / 180.0; }
		/** 度转弧度 */
		inline float DEG2RAD(const float x) { return x*M_PIf / 180.0f; }
		/** 度转弧度 */
		inline float DEG2RAD(const int x) { return x*M_PIf / 180.0f; }
		/** 弧度转度 */
		inline double RAD2DEG(const double x) { return x*180.0 / M_PI; }
		/** 弧度转度 */
		inline float RAD2DEG(const float x) { return x*180.0f / M_PIf; }

#	ifdef HAVE_LONG_DOUBLE
		/** 度转弧度 */
		inline long double DEG2RAD(const long double x) { return x*M_PIl / 180.0; }
		/** 弧度转度 */
		inline long double RAD2DEG(const long double x) { return x*180.0 / M_PIl; }
#	endif
	}
}

#endif // FBLIB_MATH_ANGLE_RADIAN_CONVERT_H_
