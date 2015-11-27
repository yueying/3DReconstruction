#ifndef MVG_FEATURE_ESTIMATION_GUIDED_MATCHING_H_
#define MVG_FEATURE_ESTIMATION_GUIDED_MATCHING_H_
#include <vector>

#include "mvg/math/numeric.h"
#include "mvg/feature/indexed_match.h"
using namespace mvg::math;

namespace mvg{
	namespace feature{
		/// Guided Matching:
		///  Use a model to find valid correspondences:
		///   Keep the best corresponding points for the given model under the
		///   user specified distance.
		template<typename ModelArg, typename ErrorArg>
		void GuidedMatching(
			const ModelArg & mod, // The model
			const Mat & left_data_points,    // The left data points
			const Mat & right_data_points,   // The right data points
			double errorTh,       // Maximal authorized error threshold
			std::vector<IndexedMatch> & vec_corresponding_index) // Ouput corresponding index
		{
			assert(left_data_points.rows() == right_data_points.rows());

			// Looking for the corresponding points that have
			//  the smallest distance (smaller than the provided Threshold)

			for (size_t i = 0; i < left_data_points.cols(); ++i) {

				double min = (std::numeric_limits<double>::max)();
				IndexedMatch match;
				for (size_t j = 0; j < right_data_points.cols(); ++j) {
					// Compute error to the model
					double err = ErrorArg::Error(
						mod,  // The model
						left_data_points.col(i), right_data_points.col(j)); // The corresponding points
					// if smaller error update corresponding index
					if (err < errorTh && err < min) {
						min = err;
						match = IndexedMatch(i, j);
					}
				}
				if (min < errorTh)  {
					// save the best corresponding index
					vec_corresponding_index.push_back(match);
				}
			}

			// Remove duplicates (when multiple points at same position exist)
			IndexedMatch::getDeduplicated(vec_corresponding_index);
		}
	}// namespace feature
} // namespace mvg

#endif // MVG_FEATURE_ESTIMATION_GUIDED_MATCHING_H_
