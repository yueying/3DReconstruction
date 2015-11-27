#ifndef MVG_FEATURE_MATCHER_BRUTE_FORCE_H_
#define MVG_FEATURE_MATCHER_BRUTE_FORCE_H_

#include <memory>
#include <iostream>
#include <vector>

#include "mvg/math/numeric.h"
#include "mvg/feature/matching_interface.h"
#include "mvg/feature/metric.h"
#include "mvg/utils/indexed_sort.h"

namespace mvg {
	namespace feature {

		/**
		 * \brief	通过暴力匹配的方式对多维数组进行匹配
		 *
		 * \tparam	Scalar	数组元素类型，默认float
		 * \tparam	Metric	匹配距离类型，默认采用欧式距离的平方
		 */
		template < typename Scalar = float, typename Metric = SquaredEuclideanDistanceSimple<Scalar> >
		class ArrayMatcherBruteForce : public ArrayMatcher < Scalar, Metric >
		{
		public:
			typedef typename Metric::ResultType DistanceType;

			ArrayMatcherBruteForce()   {}
			virtual ~ArrayMatcherBruteForce() {
				memory_mapping.reset();
			}

			/**
			* 建立匹配的结构
			*
			* \param[in] dataset   输入数据
			* \param[in] rows_num  组件数组的数目（相当于矩阵行数）
			* \param[in] dimension 每个组件数组的维度（相当于矩阵的列数）
			*
			* \return True if success.
			*/
			bool Build(const Scalar * dataset, int rows_num, int dimension) {
				if (rows_num < 1) {
					memory_mapping = std::shared_ptr< Eigen::Map<BaseMat> >(NULL);
					return false;
				}
				memory_mapping = std::shared_ptr< Eigen::Map<BaseMat> >
					(new Eigen::Map<BaseMat>((Scalar*)dataset, rows_num, dimension));
				return true;
			};

			/**
			 * 在待查询的数组中寻找最近邻的数组
			 *
			 * \param[in]   query     待查询的数组
			 * \param[out]  indice    被计算为最近邻的多维数组的索引
			 * \param[out]  distance  两个数组之间的距离.
			 *
			 * \return True if success.
			 */
			bool SearchNeighbour(const Scalar *query,
				int *indice, DistanceType *distance)
			{
				if (memory_mapping.get() != NULL)  {
					//将输入的数据转矩阵
					Eigen::Map<BaseMat> mat_query((Scalar*)query, 1, (*memory_mapping).cols());
					Metric metric;
					int array_num = (*memory_mapping).rows();
					std::vector<DistanceType> vec_dist(array_num, 0.0);
					for (int i = 0; i < array_num; ++i)  {
						// 计算距离
						vec_dist[i] = metric((Scalar*)query, (*memory_mapping).row(i).data(), (*memory_mapping).cols());
					}
					if (!vec_dist.empty())
					{
						// 找到最小距离:
						std::vector<DistanceType>::const_iterator min_iter =
							std::min_element(vec_dist.begin(), vec_dist.end());
						*indice = std::distance(
							std::vector<DistanceType>::const_iterator(vec_dist.begin()),
							min_iter);
						*distance = static_cast<DistanceType>(*min_iter);
					}
					return true;
				}
				else  {
					return false;
				}
			}

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
			bool SearchNeighbours(const Scalar *query, int query_num,
				std::vector<int> * vec_indice,
				std::vector<DistanceType> * vec_distance,
				size_t nearest_neighbor_num)
			{
				if (nearest_neighbor_num > (*memory_mapping).rows() || query_num < 1) {
					std::cerr << "Too much asked nearest neighbors" << std::endl;
					return false;
				}

				//将输入数据转矩阵
				Eigen::Map<BaseMat> mat_query((Scalar*)query, query_num, (*memory_mapping).cols());
				Metric metric;
				int array_num = (*memory_mapping).rows();
				std::vector<DistanceType> vec_dist(array_num, 0.0);
				for (int query_index = 0; query_index < query_num; ++query_index) {
					for (int i = 0; i < array_num; ++i)  {
						vec_dist[i] = metric(mat_query.row(query_index).data(),
							(*memory_mapping).row(i).data(), (*memory_mapping).cols());
					}

					// 找出N个最小距离
					int max_min_found = static_cast<int>(std::min(size_t(nearest_neighbor_num), vec_dist.size()));
					std::vector<mvg::utils::SortIndexPacketAscend< DistanceType, int> > packet_vec(vec_dist.size());
					mvg::utils::SortIndexHelper(packet_vec, &vec_dist[0], max_min_found);

					for (int i = 0; i < max_min_found; ++i) {
						vec_distance->push_back(packet_vec[i].val);
						vec_indice->push_back(packet_vec[i].index);
					}
				}
				return true;
			};

		private:
			typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> BaseMat;//!<定义动态行优先的矩阵
			std::shared_ptr< Eigen::Map<BaseMat> > memory_mapping;//!<使用内存映射，避免内存重新分配
		};

	}  // namespace feature
}  // namespace mvg


#endif // MVG_FEATURE_MATCHER_BRUTE_FORCE_H_

