#ifndef MVG_FEATURE_RANSAC_TOOLS_H_
#define MVG_FEATURE_RANSAC_TOOLS_H_

#include <cmath>

namespace mvg {
	namespace feature{

		/**
		 * \brief	计算至少取d个点计算模型才能保证点符合模型的概率
		 *
		 * \param	min_probability	总的点符合模型的概率
		 * \param	outlier_ratio  	每次从数据集中选取一个局外点的概率
		 * \param	sample_size	   	样本数目
		 *
		 * \return	至少取的d个点
		 */
		inline size_t GetNumSamples(
			double min_probability,
			double outlier_ratio,
			std::size_t sample_size)
		{
			//TODO(fengbing):再理解一下
			return static_cast<std::size_t>(std::log(1. - min_probability) /
				std::log(1. - std::pow(1. - outlier_ratio, static_cast<int>(sample_size))));
		}

		/**
		 * \brief	迭代次数估计 p = (1 - w^n)k, p表示outliers_probability，w表示inlier_ratio，n表示
		 * 			min_samples ，k表示迭代次数
		 *
		 * \param	min_samples				最小样本数，及估计模型需要的最小选定的点数
		 * \param	outliers_probability	随机取出的点不全是局内点的概率
		 * \param	inlier_ratio			每次从数据集中选取一个局内点的概率
		 *
		 * \return	参数k（迭代次数）
		 */
		inline size_t IterationsRequired(
			std::size_t min_samples,
			double outliers_probability,
			double inlier_ratio)
		{
			return static_cast<std::size_t>(
				std::log(outliers_probability) /
				std::log(1.0 - std::pow(inlier_ratio, static_cast<int>(min_samples))));
		}

	} // namespace feature
} // namespace mvg
#endif // MVG_FEATURE_RANSAC_TOOLS_H_
