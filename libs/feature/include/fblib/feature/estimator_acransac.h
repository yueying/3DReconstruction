#ifndef FBLIB_FEATURE_ESTIMATOR_ACRANSAC_H_
#define FBLIB_FEATURE_ESTIMATOR_ACRANSAC_H_

//-------------------
// ACRANSAC算法的通用实现
//-------------------
// 在文献[1]中该方法被提出，扩展到通用模型，以单应变换作为例子介绍在文献[2]中，
// 在文献[3]中对大尺度的SfM中进行了应用
//--
//  [1] Lionel Moisan, Berenger Stival,
//  A probalistic criterion to detect rigid point matches between
//  two images and estimate the fundamental matrix.
//  IJCV 04.
//--
//  [2] Lionel Moisan, Pierre Moulon, Pascal Monasse.
//  Automatic Homographic Registration of a Pair of Images,
//    with A Contrario Elimination of Outliers
//  Image Processing On Line (IPOL), 2012.
//  http://dx.doi.org/10.5201/ipol.2012.mmm-oh
//--
//  [3] Pierre Moulon, Pascal Monasse and Renaud Marlet.
//  Adaptive Structure from Motion with a contrario mode estimation.
//  In 11th Asian Conference on Computer Vision (ACCV 2012)
//--


#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>
#include <limits>
#include <iostream>

#include "fblib/feature/random_sampling.h"

namespace fblib {
	namespace feature{

		/// logarithm (base 10) of binomial coefficient
		static double logcombi(size_t k, size_t n)
		{
			if (k >= n || k <= 0) return(0.0);
			if (n - k < k) k = n - k;
			double r = 0.0;
			for (size_t i = 1; i <= k; i++)
				r += log10((double)(n - i + 1)) - log10((double)i);

			return r;
		}

		/// tabulate logcombi(.,n)
		template<typename Type>
		static void makelogcombi_n(size_t n, std::vector<Type> & l)
		{
			l.resize(n + 1);
			for (size_t k = 0; k <= n; k++)
				l[k] = static_cast<Type>(logcombi(k, n));
		}

		/// tabulate logcombi(k,.)
		template<typename Type>
		static void makelogcombi_k(size_t k, size_t nmax, std::vector<Type> & l)
		{
			l.resize(nmax + 1);
			for (size_t n = 0; n <= nmax; n++)
				l[n] = static_cast<Type>(logcombi(k, n));
		}

		/// Distance and associated index
		typedef std::pair<double, size_t> ErrorIndex;

		/// Find best NFA and its index wrt square error threshold in e.
		static ErrorIndex bestNFA(
			int startIndex, //number of point required for estimation
			double logalpha0,
			const std::vector<ErrorIndex>& e,
			double loge0,
			double maxThreshold,
			const std::vector<float> &logc_n,
			const std::vector<float> &logc_k,
			double multError = 1.0)
		{
			ErrorIndex bestIndex(std::numeric_limits<double>::infinity(), startIndex);
			const size_t n = e.size();
			for (size_t k = startIndex + 1; k <= n && e[k - 1].first <= maxThreshold; ++k) {
				double logalpha = logalpha0 + multError * log10(e[k - 1].first + (std::numeric_limits<float>::min)());
				ErrorIndex index(loge0 +
					logalpha * (double)(k - startIndex) +
					logc_n[k] +
					logc_k[k], k);

				if (index.first < bestIndex.first)
					bestIndex = index;
			}
			return bestIndex;
		}

		/// Pick a random sample
		/// \param sample_size The size of the sample.
		/// \param vec_index  The possible data indices.
		/// \param sample The random sample of sample_size indices (output).
		static void UniformSample(int sample_size,
			const std::vector<size_t> &vec_index,
			std::vector<size_t> *sample)
		{
			sample->resize(sample_size);
			RandomSample(sample_size, vec_index.size(), sample);
			for (int i = 0; i < sample_size; ++i)
				(*sample)[i] = vec_index[(*sample)[i]];
		}

		/**
		 * @brief ACRANSAC routine (ErrorThreshold, NFA)
		 *
		 * @param[in] kernel model and metric object
		 * @param[out] vec_inliers points that fit the estimated model
		 * @param[in] iter_num maximum number of consecutive iterations
		 * @param[out] model returned model if found
		 * @param[in] precision upper bound of the precision (squared error)
		 * @param[in] is_verbose display console log
		 *
		 * @return (errorMax, minNFA)
		 */
		template<typename Kernel>
		std::pair<double, double> ACRANSAC(const Kernel &kernel,
			std::vector<size_t> & vec_inliers,
			size_t iter_num = 1024,
			typename Kernel::Model * model = NULL,
			double precision = std::numeric_limits<double>::infinity(),
			bool is_verbose = false)
		{
			vec_inliers.clear();

			const size_t sample_size = Kernel::MINIMUM_SAMPLES;
			const size_t nData = kernel.NumSamples();
			if (nData <= (size_t)sample_size)
				return std::make_pair(0.0, 0.0);

			const double maxThreshold = (precision == std::numeric_limits<double>::infinity()) ?
				std::numeric_limits<double>::infinity() :
				precision * kernel.normalizer2()(0, 0) * kernel.normalizer2()(0, 0);

			std::vector<ErrorIndex> vec_residuals(nData); // [residual,index]
			std::vector<size_t> vec_sample(sample_size); // Sample indices

			// Possible sampling indices (could change in the optimization phase)
			std::vector<size_t> vec_index(nData);
			for (size_t i = 0; i < nData; ++i)
				vec_index[i] = i;

			// Precompute log combi
			double loge0 = log10((double)Kernel::MAX_MODELS * (nData - sample_size));
			std::vector<float> vec_logc_n, vec_logc_k;
			makelogcombi_n(nData, vec_logc_n);
			makelogcombi_k(sample_size, nData, vec_logc_k);

			// Output parameters
			double minNFA = std::numeric_limits<double>::infinity();
			double errorMax = std::numeric_limits<double>::infinity();

			// Reserve 10% of iterations for focused sampling
			size_t nIterReserve = iter_num / 10;
			iter_num -= nIterReserve;

			// Main estimation loop.
			for (size_t iter = 0; iter < iter_num; ++iter) {
				UniformSample(sample_size, vec_index, &vec_sample); // Get random sample

				std::vector<typename Kernel::Model> vec_models; // Up to max_models solutions
				kernel.Fit(vec_sample, &vec_models);

				// Evaluate models
				bool better = false;
				for (size_t k = 0; k < vec_models.size(); ++k)  {
					// Residuals computation and ordering
					for (size_t i = 0; i < nData; ++i)  {
						double error = kernel.Error(i, vec_models[k]);
						vec_residuals[i] = ErrorIndex(error, i);
					}
					std::sort(vec_residuals.begin(), vec_residuals.end());

					// Most meaningful discrimination inliers/outliers
					ErrorIndex best = bestNFA(sample_size,
						kernel.logalpha0(),
						vec_residuals,
						loge0,
						maxThreshold,
						vec_logc_n,
						vec_logc_k,
						kernel.multError());

					if (best.first < minNFA /*&& vec_residuals[best.second-1].first < errorMax*/)  {
						// A better model was found
						better = true;
						minNFA = best.first;
						vec_inliers.resize(best.second);
						for (size_t i = 0; i < best.second; ++i)
							vec_inliers[i] = vec_residuals[i].second;
						errorMax = vec_residuals[best.second - 1].first; // Error threshold
						if (model) *model = vec_models[k];

						if (is_verbose)  {
							std::cout << "  nfa=" << minNFA
								<< " inliers=" << best.second
								<< " precisionNormalized=" << errorMax
								<< " precision=" << kernel.unormalizeError(errorMax)
								<< " (iter=" << iter;
							std::cout << ",sample=";
							std::copy(vec_sample.begin(), vec_sample.end(),
								std::ostream_iterator<size_t>(std::cout, ","));
							std::cout << ")" << std::endl;
						}
					}
				}

				// ACRANSAC optimization: draw samples among best set of inliers so far
				if ((better && minNFA < 0) || (iter + 1 == iter_num && nIterReserve)) {
					if (vec_inliers.empty()) { // No model found at all so far
						iter_num++; // Continue to look for any model, even not meaningful
						nIterReserve--;
					}
					else {
						// ACRANSAC optimization: draw samples among best set of inliers so far
						vec_index = vec_inliers;
						if (nIterReserve) {
							iter_num = iter + 1 + nIterReserve;
							nIterReserve = 0;
						}
					}
				}
			}

			if (minNFA >= 0)
				vec_inliers.clear();

			if (!vec_inliers.empty())
			{
				if (model)
					kernel.Unnormalize(model);
				errorMax = kernel.unormalizeError(errorMax);
			}

			return std::make_pair(errorMax, minNFA);
		}


	} // namespace feature
} // namespace fblib
#endif // FBLIB_FEATURE_ESTIMATOR_ACRANSAC_H_
