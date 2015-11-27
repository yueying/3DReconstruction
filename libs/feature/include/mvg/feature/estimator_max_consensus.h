#ifndef MVG_FEATURE_ESTIMATION_MAX_CONSENSUS_H_
#define MVG_FEATURE_ESTIMATION_MAX_CONSENSUS_H_

#include <limits>
#include <vector>

#include "mvg/feature/random_sampling.h"

namespace mvg {
	namespace feature{

		/**
		 * \brief 基本的RANSAC实现，没有提供取样个数，迭代次数的选择

		 * 主要确定 :
		 * 1. 模型
		 * 2. 计算模型所需要的最小的样本数
		 * 3. 给出方法将样本转化为模型
		 * 4. 给出方法计算样本与模型之间的误差
		 *
		 * 1. Kernel::Model
		 * 2. Kernel::MINIMUM_SAMPLES
		 * 3. Kernel::Fit(vector<int>, vector<Kernel::Model> *)
		 * 4. Kernel::Error(Model, int) -> error
		 *
		 */
		template<typename Kernel, typename Scorer>
		typename Kernel::Model MaxConsensus(const Kernel &kernel,
			const Scorer &scorer,
			std::vector<size_t> *best_inliers = NULL, size_t max_iteration = 1024) {

			// 模型估计需要的最小点数
			const size_t min_samples = Kernel::MINIMUM_SAMPLES;
			// 得到样本总数
			const size_t total_samples = kernel.NumSamples();

			double best_cost = std::numeric_limits<double>::infinity();
			typename Kernel::Model best_model;

			// 检测是否有足够的点对模型进行测试
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
			for (size_t iteration = 0; iteration < max_iteration; ++iteration) {
				UniformSample(min_samples, total_samples, &sample);

				std::vector<typename Kernel::Model> models;
				kernel.Fit(sample, &models);

				// 计算每次残差，选择最小残差，确定模型
				for (size_t i = 0; i < models.size(); ++i) {
					std::vector<size_t> inliers;
					double cost = scorer.Score(kernel, models[i], all_samples, &inliers);

					if (cost < best_cost) {
						best_cost = cost;
						best_model = models[i];
						if (best_inliers) {
							best_inliers->swap(inliers);
						}
					}
				}
			}
			return best_model;
		}

	} // namespace feature
} // namespace mvg

#endif // MVG_FEATURE_ESTIMATION_MAX_CONSENSUS_H_
