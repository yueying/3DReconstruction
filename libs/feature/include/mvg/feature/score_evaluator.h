#ifndef MVG_FEATURE_SCORE_EVALUATOR_H_
#define MVG_FEATURE_SCORE_EVALUATOR_H_

namespace mvg {
	namespace feature{

		/**
		 * \brief	通过给定模型及样本对模型进行评判
		 *
		 * \tparam	Kernel	求解内核类型比如线程内核
		 */
		template<typename Kernel>
		class ScorerEvaluator {
		public:

			/**
			 * \brief	通过给定模型及样本对模型进行评判.
			 *
			 * \param	threshold	评判阈值大小
			 */
			ScorerEvaluator(double threshold) : threshold_(threshold) {}

			/**
			* \brief	通过给定模型及样本对模型进行评判.
			*/
			template <typename T>
			double Score(const Kernel &kernel,
				const typename Kernel::Model &model,
				const std::vector<T> &samples,
				std::vector<T> *inliers) const
			{
				double cost = 0.0;
				for (size_t j = 0; j < samples.size(); ++j) {
					double error = kernel.Error(samples[j], model);
					if (error < threshold_) {
						cost += error;
						inliers->push_back(samples[j]);
					}
					else {
						cost += threshold_;
					}
				}
				return cost;
			}
		private:
			double threshold_;
		};

	} // namespace feature
} // namespace mvg

#endif // MVG_FEATURE_SCORE_EVALUATOR_H_
