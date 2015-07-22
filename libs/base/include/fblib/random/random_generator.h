/*******************************************************************************
 * 文件： random_generator.h
 * 时间： 2014/12/31 14:26
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 随机数生成，这边根据c++11标准进行封装
 *
********************************************************************************/
#ifndef FBLIB_RANDOM_RANDOM_GENERATOR_H_
#define FBLIB_RANDOM_RANDOM_GENERATOR_H_

#include <fblib/base/link_pragmas.h>
#include <random>

namespace fblib
{
	namespace random
	{
		class BASE_IMPEXP RandomGenerator
		{
		public:
			RandomGenerator() : engine(), random_device() { seed(); }
			void seed();
			int randomIntWithinRange(int imin, int imax);
			float randomFloatWithinRange(float fmin, float fmax);
		protected:
			std::random_device random_device;
			std::mt19937_64 engine;
		};

		extern BASE_IMPEXP RandomGenerator randomGenerator;//全局声明
	}
}

#endif // FBLIB_RANDOM_RANDOM_GENERATOR_H_