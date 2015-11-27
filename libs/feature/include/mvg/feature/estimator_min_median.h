#ifndef MVG_FEATURE_ESTIMATION_MIN_MEDIAN_H_
#define MVG_FEATURE_ESTIMATION_MIN_MEDIAN_H_

#include <algorithm>
#include <limits>
#include <vector>

#include "mvg/feature/random_sampling.h"
#include "mvg/feature/estimator_ransac_tools.h"

namespace mvg {
	namespace feature{

	   /* \brief 最小中值方法
		* \details 通过给出数据残差平方中值达到最小的模型参数，过程与RANSAC类型，不同的
		* 是不再将数据分为内点和外点，最小化在整个数据集上进行。这种方法的优点是不需要内点和
		* 外点的先验知识，也就是不需要划分内点和外点的距离阈值的先验知识。
		* 注：1、由于在整个数据集上取中值，当数据集有大于50%的错误数据点时，这种方法不可能给出正确估计结果。
		* 2、当数据测量误差服从高斯分布时，最小中值法的效率非常差（效率为给定方法给出
		* 的方差与理论上能达到的方差之比）为了弥补这个不足，可以再执行最小中值方法之后，再对
		* 估计结果进一步进行加权最优化
		* 
		* 参考 : Z. Zhang. Determining The Epipolar Geometry And Its Uncertainty  A Review. IJCV 1998
		*/
		template <typename Kernel>
		double LeastMedianOfSquares(const Kernel &kernel,
			typename Kernel::Model * model = NULL,
			double* outlier_threshold = NULL,
			double outlier_ratio = 0.5,
			double min_probability = 0.99)
		{
			const size_t min_samples = Kernel::MINIMUM_SAMPLES;
			const size_t total_samples = kernel.NumSamples();

			std::vector<double> residuals(total_samples); // 用来存放残差
			std::vector<size_t> vec_sample(min_samples);

			double best_median = std::numeric_limits<double>::max();

			// 计算可能的采样个数
			const size_t N = (min_samples < total_samples) ?
				GetNumSamples(min_probability, outlier_ratio, min_samples) : 0;

			for (size_t i = 0; i < N; i++) {
				// 随机取样，得到样本索引
				UniformSample(min_samples, total_samples, &vec_sample);

				// 参数估计，将结果保存到一个vector中
				std::vector<typename Kernel::Model> models;
				kernel.Fit(vec_sample, &models);

				// 对整个数据集进行计算每个残差，计算残差中值
				for (size_t k = 0; k < models.size(); ++k) {
					// 计算残差值 :
					for (size_t l = 0; l < total_samples; ++l) {
						double error = kernel.Error(l, models[k]);
						residuals[l] = error;
					}

					// 计算中值
					std::vector<double>::iterator iter_median = residuals.begin() +
						std::size_t(total_samples*(1. - outlier_ratio));
					std::nth_element(residuals.begin(), iter_median, residuals.end());
					double median = *iter_median;

					// 保存最好的解决方案
					if (median < best_median) {
						best_median = median;
						if (model) (*model) = models[k];
					}
				}
			}

			// 保证达到 ( 高斯误差分布 ) 最大然似估计的同样效率的校正系数
			static const double ICDF[21] = {
				1.4e16, 15.94723940, 7.957896558, 5.287692054,
				3.947153876, 3.138344200, 2.595242369, 2.203797543,
				1.906939402, 1.672911853, 1.482602218, 1.323775627,
				1.188182950, 1.069988721, 0.9648473415, 0.8693011162,
				0.7803041458, 0.6946704675, 0.6079568319, 0.5102134568,
				0.3236002672
			};

			// 设定阈值
			if (outlier_threshold) {
				double sigma = ICDF[int((1. - outlier_ratio)*20.)] *
					(1. + 5. / double(total_samples - min_samples));// 5/(N-n) 是为了补偿小数据集的效应
				*outlier_threshold = (double)(sigma * sigma * best_median * 4.);//TODO:这边是否需要修改
				if (N == 0) *outlier_threshold = std::numeric_limits<double>::max();
			}

			return best_median;
		}

	} // namespace feature
} // namespace mvg

#endif // MVG_FEATURE_ESTIMATION_MIN_MEDIAN_H_
