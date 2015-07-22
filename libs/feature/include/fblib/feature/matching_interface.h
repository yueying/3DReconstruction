#ifndef FBLIB_FEATURE_MATCHING_INTERFACE_H_
#define FBLIB_FEATURE_MATCHING_INTERFACE_H_

#include <vector>
#include "fblib/math/numeric.h"

namespace fblib {
	namespace feature {

		/**
		 * \brief	多维数组的匹配
		 *
		 * \tparam	Scalar	数组元素类型
		 * \tparam	Metric	匹配度量距离的类型
		 */
		template < typename Scalar, typename Metric >
		class ArrayMatcher
		{
		public:
			typedef typename Metric::ResultType DistanceType;
			typedef Metric MetricT;

			ArrayMatcher() {}
			virtual ~ArrayMatcher() {};

			/**
			 * 建立匹配的结构
			 *
			 * \param[in] dataset   输入数据
			 * \param[in] rows_num  组件的数目（相当于矩阵行数）
			 * \param[in] dimension 每个组件的维度（相当于矩阵的列数）
			 *
			 * \return True if success.
			 */
			virtual bool Build(const Scalar *dataset, int rows_num, int dimension) = 0;

			/**
			* 在待查询的数组中寻找最近邻的数组
			*
			* \param[in]   query     待查询的数组
			* \param[out]  indice    被计算为最近邻的多维数组的索引
			* \param[out]  distance  两个数组之间的距离.
			*
			* \return True if success.
			*/
			virtual bool SearchNeighbour(const Scalar *query,
				int *indice, DistanceType *distance) = 0;

			/**
			* \brief	在待查询的数组中寻找多个最近邻的数组(k近邻)
			*
			* \param [in]	query		 	待查询的所有数组.
			* \param [in]	query_num		匹配查询的数组数目
			* \param [out]	vec_indice   	被计算为最近邻的多维数组的索引,这边为最近邻的个数
			* \param [out]	vec_distance	匹配数组之间的距离，这边为最近邻的个数
			* \param [out]	nearest_neighbor_num	最近邻的数目k
			*
			* \return	True if success.
			*/
			virtual bool SearchNeighbours(const Scalar * query, int query_num,
				std::vector<int> * vec_indice,
				std::vector<DistanceType> * vec_distance,
				size_t nearest_neighbor_num) = 0;
		};

	}  // namespace feature
}  // namespace fblib

#endif // FBLIB_FEATURE_MATCHING_INTERFACE_H_
