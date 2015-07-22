#ifndef FBLIB_FEATURE_MATCHING_FILTERS_H_
#define FBLIB_FEATURE_MATCHING_FILTERS_H_

#include <algorithm>
#include <cassert>
#include <iterator>
#include <set>
#include <vector>

#include "fblib/feature/indexed_match.h"

namespace fblib {
	namespace feature {

		/**
		  * 通过最近距离除以次近的距离少于某个比例阈值来进行过滤 ( a < ratio * b) :
		  * 降低这个比例阈值，匹配点数目会减少，但更加稳定
		  *
		  * \param[in]  first    距离序列迭代的开始
		  * \param[in]  last     距离序列迭代的结束
		  * \param[in]  nearest_neighbor_num       在迭代序列中寻找最近邻的数目(最小为2)
		  * \param[out] vec_after_ratio_index 经过比率阈值过滤之后被计算为最近邻的多维数组的索引
		  * \param[in]  ratio            比率阈值(默认值0.6f)
		  *
		  * \return void.
		  */
		template <typename DataInputIterator>
		static void DistanceRatioFilter(DataInputIterator first,
			DataInputIterator last,
			int nearest_neighbor_num, 
			std::vector<int> &vec_after_ratio_index, 
			float ratio = 0.6f) 
		{
			assert(nearest_neighbor_num >= 2);

			vec_after_ratio_index.clear();
			const size_t n = std::distance(first, last);
			DataInputIterator iter = first;
			for (size_t i = 0; i < n / nearest_neighbor_num; ++i, std::advance(iter, nearest_neighbor_num))
			{
				DataInputIterator iter2 = iter;
				std::advance(iter2, 1);
				//不管nearest_neighbor_num的数目，只考虑最近邻与次近邻的比值
				if ((*iter) < ratio * (*iter2))
					vec_after_ratio_index.push_back(static_cast<int>(i));
			}
		}

		/**
		  * Symmetric matches filtering :
		  * Suppose matches from dataset A to B stored in vec_matches
		  * Suppose matches from dataset B to A stored in vec_reversematches
		  * A matches is kept if (i == vec_reversematches[vec_matches[i]])
		  * If nearest_neighbor_num > 1 => Only the major matches are considered.
		  *
		  * \param[in]  first    matches from A to B.
		  * \param[in]  last     matches from B to A.
		  * \param[in]  nearest_neighbor_num       Number of neighbor matches.
		  * \param[out] vec_goodIndex  Indexes that respect Symmetric matches
		  *
		  * \return void.
		  */
		// TODO
		static void SymmetricMatches(const std::vector<int> & vec_matches,
			const std::vector<int> & vec_reversematches,
			int nearest_neighbor_num,
			std::vector<int> & vec_goodIndex)
		{
			assert(nearest_neighbor_num >= 1);

			int index = 0;
			for (size_t i = 0; i < vec_matches.size(); i += nearest_neighbor_num, ++index)
			{
				// Add the match only if we have a symmetric result.
				if (index == vec_reversematches[vec_matches[i] * nearest_neighbor_num])  {
					vec_goodIndex.push_back(index);
				}
			}
		}

		/**
		 * \brief	计算两个迭代序列内元素的交集，注意两个迭代序列必须有值
		 *
		 * \tparam	Iterator	迭代器的类型
		 * \tparam	Type		存放元素的类型
		 * \param [in]	iter_a_start 	A序列迭代开始
		 * \param [in]	iter_a_end   	A序列迭代结束
		 * \param [in]	iter_b_start 	B序列迭代开始
		 * \param [in]	iter_b_end   	B序列迭代结束
		 * \param [out]	vec_out	返回两个迭代序列元素的交集
		 */
		template <typename Iterator, typename Type>
		static void IntersectMatches(Iterator iter_a_start, Iterator iter_a_end,
			Iterator iter_b_start, Iterator iter_b_end,
			std::vector<Type> & vec_out)
		{
			//通过STL中set_intersection进行计算交集
			std::set<Type> intersect;
			std::set_intersection(iter_a_start, iter_a_end,
				iter_b_start, iter_b_end,
				std::inserter(intersect, intersect.begin()));

			vec_out = std::vector<Type>(intersect.begin(), intersect.end());
		}

		enum eMatchFilter
		{
			MATCHFILTER_SYMMETRIC = 1,
			MATCHFILTER_NNDISTANCERATIO = 2,
			MATCHFILER_SYM_AND_NNDISTANCERATIO = MATCHFILTER_SYMMETRIC | MATCHFILTER_NNDISTANCERATIO
		};

		static void Filter(int nearest_neighbor_num,
			const std::vector<int> & vec_Matches01,
			const std::vector<float> & vec_distance01,
			const std::vector<int> & vec_Matches10,
			const std::vector<float> & vec_distance10,
			std::vector<IndexedMatch> & vec_outIndex,
			eMatchFilter matchFilter,
			float fNNDistanceRatio = 0.6f)
		{
			std::vector<int> vec_symmetricIndex, vec_NNDistRatioIndexes;

			if (matchFilter == MATCHFILTER_SYMMETRIC ||
				matchFilter == MATCHFILER_SYM_AND_NNDISTANCERATIO)
			{
				SymmetricMatches(vec_Matches01,
					vec_Matches10,
					nearest_neighbor_num,
					vec_symmetricIndex);
			}

			if (matchFilter == MATCHFILTER_NNDISTANCERATIO ||
				matchFilter == MATCHFILER_SYM_AND_NNDISTANCERATIO)
			{
				if (nearest_neighbor_num == 1)
				{
					vec_NNDistRatioIndexes = vec_Matches01;
				}

				DistanceRatioFilter(vec_distance01.begin(), // distance start
					vec_distance01.end(),  // distance end
					nearest_neighbor_num, // Number of neighbor in iterator sequence (minimum required 2)
					vec_NNDistRatioIndexes, // output (index that respect Lowe Ratio)
					fNNDistanceRatio);
			}

			switch (matchFilter)
			{
			case MATCHFILTER_NNDISTANCERATIO:

				for (size_t i = 0; i < vec_NNDistRatioIndexes.size() - 1 && vec_NNDistRatioIndexes.size()>0; ++i)
				{
					vec_outIndex.push_back(IndexedMatch(vec_NNDistRatioIndexes[i], vec_Matches01[vec_NNDistRatioIndexes[i] * nearest_neighbor_num]));
				}

				break;

			case MATCHFILTER_SYMMETRIC:

				for (size_t i = 0; i < vec_symmetricIndex.size() - 1 && vec_symmetricIndex.size()>0; ++i)
				{
					vec_outIndex.push_back(IndexedMatch(vec_symmetricIndex[i], vec_Matches01[vec_symmetricIndex[i] * nearest_neighbor_num]));
				}

				break;

			case MATCHFILER_SYM_AND_NNDISTANCERATIO:

				std::vector<int> vec_indexes;
				//-- Compute the intersection of the two vector
				IntersectMatches(vec_symmetricIndex.begin(), vec_symmetricIndex.end(),
					vec_NNDistRatioIndexes.begin(), vec_NNDistRatioIndexes.end(),
					vec_indexes);

				for (size_t i = 0; i < vec_indexes.size() - 1 && vec_indexes.size()>0; ++i)
					vec_outIndex.push_back(IndexedMatch(vec_indexes[i], vec_Matches01[vec_indexes[i] * nearest_neighbor_num]));

				break;
			}

			// Remove multi-index
  {
	  std::sort(vec_outIndex.begin(), vec_outIndex.end());
	  std::vector<IndexedMatch>::iterator end = std::unique(vec_outIndex.begin(), vec_outIndex.end());
	  if (end != vec_outIndex.end()) {
		  vec_outIndex.erase(end, vec_outIndex.end());
	  }
  }
		}

	}  // namespace feature
}  // namespace fblib


#endif // FBLIB_FEATURE_MATCHING_FILTERS_H_
