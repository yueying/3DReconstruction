#ifndef FBLIB_FEATURE_ESTIMATION_RAND_SAMPLING_H_
#define FBLIB_FEATURE_ESTIMATION_RAND_SAMPLING_H_

#include <vector>
#include <stdlib.h>

namespace fblib {
	namespace feature{

		using namespace std;

		/**
		 * \brief	在整数[0, total)范围内随机采样，注意如果采样数接近区间，则运行时间值得考虑。
		 * 			计算过程中随机采样获得的数跟前面的数进行比较，判断是否已经存在，
		 * 			因此考虑耗时，采样数目num_samples应该较小
		 *
		 * \param	num_samples	   	采样的数目
		 * \param	total_samples  	可用的样本数
		 * \param [in,out]	samples	返回num_samples采样数目在这个区间[0, total_samples)
		 */
		static void UniformSample(size_t num_samples, size_t total_samples, std::vector<size_t> *samples)
		{
			samples->resize(0);
			while (samples->size() < num_samples) {
				size_t sample = size_t(rand() % total_samples);
				bool is_found = false;
				for (size_t j = 0; j < samples->size(); ++j) {
					is_found = (*samples)[j] == sample;
					if (is_found) { //采样已经存在
						break;
					}
				}
				if (!is_found) {
					samples->push_back(sample);
				}
			}
		}

		/**
		 * \brief	在[0:n-1]的范围内得到X个排好序的随机数，采样插入排序的思想
		 *
		 * \param	X			   	要获得的X个随机数
		 * \param	n			   	随机数获取的范围
		 * \param [in,out]	samples	返回获取的随机数
		 */
		static void RandomSample(size_t X, size_t n, std::vector<size_t> *samples)
		{
			samples->resize(X);
			for (size_t i = 0; i < X; ++i) {
				size_t r = (rand() >> 3) % (n - i), j;
				for (j = 0; j < i && r >= (*samples)[j]; ++j)
					++r;
				size_t j0 = j;
				for (j = i; j > j0; --j)
					(*samples)[j] = (*samples)[j - 1];
				(*samples)[j0] = r;
			}
		}

	} // namespace feature
} // namespace fblib
#endif // FBLIB_FEATURE_ESTIMATION_RAND_SAMPLING_H_
