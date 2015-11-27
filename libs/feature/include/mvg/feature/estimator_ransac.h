#ifndef MVG_FEATURE_ESTIMATION_RANSAC_H_
#define MVG_FEATURE_ESTIMATION_RANSAC_H_

#include "mvg/feature/random_sampling.h"
#include "mvg/feature/estimator_ransac_tools.h"
#include <limits>
#include <vector>

namespace mvg {
	namespace feature{

		/* \brief 随机抽样一致性算法实现
		*  \details 多次迭代找到更多的inliers，以确定模型，主要确定如下：		
		* 1. 模型
		* 2. 计算模型所需要的最小的样本数
		* 3. 给出方法将样本转化为模型
		* 4. 给出方法计算样本与模型之间的误差
		*
		* 1. Kernel::Model
		* 2. Kernel::MINIMUM_SAMPLES
		* 3. Kernel::Fit(vector<int>, vector<Kernel::Model> *)
		* 4. Kernel::Error(Model, int) -> error
		*/
		template<typename Kernel, typename Scorer>
		typename Kernel::Model RANSAC(
			const Kernel &kernel,
			const Scorer &scorer,
			std::vector<size_t> *best_inliers = NULL,
			double *best_score = NULL,
			double outliers_probability = 1e-2)
		{
			assert(outliers_probability < 1.0);
			assert(outliers_probability > 0.0);

			size_t iteration = 0;
			const size_t min_samples = Kernel::MINIMUM_SAMPLES;
			const size_t total_samples = kernel.NumSamples();

			size_t max_iterations = 100;//迭代次数后续可以进行估计
			const size_t really_max_iterations = 4096;//最大迭代次数

			size_t best_num_inliers = 0;
			double best_cost = std::numeric_limits<double>::infinity();
			double best_inlier_ratio = 0.0;
			typename Kernel::Model best_model;

			// 验证是否有足够的点来确保模型的计算
			if (total_samples < min_samples) {
				if (best_inliers) {
					best_inliers->resize(0);
				}
				return best_model;
			}

			// 在特征估计中，评判标准建立在所有样本中
			std::vector<size_t> all_samples;
			for (size_t i = 0; i < total_samples; ++i) {
				all_samples.push_back(i);
			}

			std::vector<size_t> sample;
			for (iteration = 0;
				iteration < max_iterations &&
				iteration < really_max_iterations; ++iteration) {
				UniformSample(min_samples, total_samples, &sample);

				std::vector<typename Kernel::Model> models;
				kernel.Fit(sample, &models);

				// 将样本带入模型计算误差，判断误差是否小于前一次误差，小于则替代
				for (size_t i = 0; i < models.size(); ++i) {
					std::vector<size_t> inliers;
					double cost = scorer.Score(kernel, models[i], all_samples, &inliers);

					if (cost < best_cost) {
						best_cost = cost;
						best_inlier_ratio = inliers.size() / double(total_samples);
						best_num_inliers = inliers.size();
						best_model = models[i];
						if (best_inliers) {
							best_inliers->swap(inliers);
						}
					}
					if (best_inlier_ratio) {
						max_iterations = IterationsRequired(min_samples,
							outliers_probability,
							best_inlier_ratio);
					}
				}
			}
			if (best_score)
				*best_score = best_cost;
			return best_model;
		}


	} // namespace feature
} // namespace mvg

#endif // MVG_FEATURE_ESTIMATION_RANSAC_H_
