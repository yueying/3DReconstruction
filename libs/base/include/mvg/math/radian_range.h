/*******************************************************************************
 * 文件： radian_range.h
 * 时间： 2014/12/05 9:48
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 将弧度转换到[0,2pi]或[-pi,pi]，只是针对浮点型
 *
********************************************************************************/
#ifndef MVG_MATH_RADIAN_RANGE_H_
#define MVG_MATH_RADIAN_RANGE_H_

#include <cmath>

namespace mvg
{
	namespace math
	{
		/** 将弧度转到[0,2pi]
		*/
		template <class T>
		inline void RadianTo2PiInPlace(T &a)
		{
			bool was_neg = a<0;
			a = fmod(a, static_cast<T>(2.0*M_PI));//fmod求余
			if (was_neg) a += static_cast<T>(2.0*M_PI);
		}

		/** 将弧度转换到[0,2pi]
		*/
		template <class T>
		inline T RadianTo2Pi(T a)
		{
			RadianTo2PiInPlace(a);
			return a;
		}

		/** 将弧度转换到[-pi,pi]
		*/
		template <class T>
		inline T RadianToPi(T a)
		{
			return RadianTo2Pi(a + static_cast<T>(M_PI)) - static_cast<T>(M_PI);
		}

		/** 将弧度转换到[-pi,pi]
		*/
		template <class T>
		inline void RadianToPiInPlace(T &a)
		{
			a = RadianToPi(a);
		}

		/** 修改一个序列的角度
		*/
		template <class VECTOR>
		void RadianTo2PiInSequence(VECTOR &x)
		{
			const size_t N = x.size();
			for (size_t i = 0; i<N; i++)
			{
				mvg::math::RadianToPiInPlace(x[i]); // 确保范围[-pi,pi].
				if (!i) continue;
				double Ap = x[i] - x[i - 1];
				if (Ap>M_PI)  x[i] -= 2.*M_PI;
				if (Ap<-M_PI) x[i] += 2.*M_PI;
			}
		}

	} // End of math namespace
} // End of namespace

#endif // MVG_MATH_RADIAN_RANGE_H_