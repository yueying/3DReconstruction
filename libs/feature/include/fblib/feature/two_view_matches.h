#ifndef FBLIB_FEATURE_TWO_VIEW_MATCHES_H_
#define FBLIB_FEATURE_TWO_VIEW_MATCHES_H_

#include "fblib/feature/matching_filters.h"

namespace fblib{
	namespace feature{

		template< typename DescriptorsT, typename MatcherT>
		void GetPutativesMatches(
			const std::vector<DescriptorsT > &left_descriptors,
			const std::vector<DescriptorsT > &right_descriptors,
			const float nearest_neighbor_distance_ratio,
			std::vector<IndexedMatch> &vec_putative_matches)
		{
			typedef typename DescriptorsT::bin_type DescTbin;

			const int kNearestNeighborDistance = 2; // 对每一个左边的描述子寻找前两个最近距离
			std::vector<int> vec_indice;
			std::vector<typename MatcherT::MetricT::ResultType> vec_distance;

			MatcherT matcher;
			const DescTbin* kLeftDescriptorsReinterpret =
				reinterpret_cast<const DescTbin *>(&left_descriptors[0]);

			const DescTbin* kRightDescriptorsReinterpret =
				reinterpret_cast<const DescTbin *>(&right_descriptors[0]);

			matcher.Build(kLeftDescriptorsReinterpret, left_descriptors.size(), DescriptorsT::kStaticSize);
			matcher.SearchNeighbours(kRightDescriptorsReinterpret, right_descriptors.size(), &vec_indice, &vec_distance, kNearestNeighborDistance);

			// 通过距离比进行过滤
			std::vector<int> vec_lowe_ratio_indexes;
			DistanceRatioFilter(vec_distance.begin(), vec_distance.end(),
				kNearestNeighborDistance, 
				vec_lowe_ratio_indexes, 
				nearest_neighbor_distance_ratio);

			// 得到特征对应索引
			for (size_t k = 0; k < vec_lowe_ratio_indexes.size() - 1
				&& vec_lowe_ratio_indexes.size() > 0; ++k) {
				vec_putative_matches.push_back(
					IndexedMatch(vec_indice[vec_lowe_ratio_indexes[k] * kNearestNeighborDistance],
					vec_lowe_ratio_indexes[k]));
			}
		}
	}
}

#endif // FBLIB_FEATURE_TWO_VIEW_MATCHES_H_
