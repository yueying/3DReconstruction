#ifndef MVG_FEATURE_MATCHER_KDTREE_FLANN_H_
#define MVG_FEATURE_MATCHER_KDTREE_FLANN_H_

#include <memory>
#include "flann/flann.h"
#include "mvg/feature/matching_interface.h"

namespace mvg {
	namespace feature  {

		/** 实现数组的FLANN的KD树匹配，具体参考http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN
		* 默认使用L2范数进行度量(flann::L2<Scalar>)
		* 这里不计算距离即不进行开根处理，由于单调性，可以直接计算平方和
		*/
		template < typename Scalar = float, typename  Metric = flann::L2<Scalar> >
		class ArrayMatcherKdtreeFlann : public ArrayMatcher < Scalar, Metric >
		{
		public:
			typedef typename Metric::ResultType DistanceType;

			ArrayMatcherKdtreeFlann() {}

			virtual ~ArrayMatcherKdtreeFlann()  {
				_datasetM.reset();
				_index.reset();
			}

			/**
			 * 构建匹配的结构
			 *
			 * \param[in] dataset  输入数据
			 * \param[in] rows_num  组件的数目
			 * \param[in] dimension 在数据集中每一行数据的长度
			 *
			 * \return True if success.
			 */
			bool Build(const Scalar *dataset, int rows_num, int dimension)  {

				if (rows_num > 0)
				{
					_dimension = dimension;
					//-- Build Flann Matrix container (map to already allocated memory)
					_datasetM = std::shared_ptr< flann::Matrix<Scalar> >(
						new flann::Matrix<Scalar>((Scalar*)dataset, rows_num, dimension));

					//-- Build FLANN index
					_index = std::shared_ptr< flann::Index<Metric> >(
						new flann::Index<Metric>(*_datasetM, flann::KDTreeIndexParams(4)));
					(*_index).buildIndex();

					return true;
				}
				return false;
			}

			/**
			 * Search the nearest Neighbor of the scalar array query.
			 *
			 * \param[in]   query     The query array
			 * \param[out]  indice    The indice of array in the dataset that
			 *  have been computed as the nearest array.
			 * \param[out]  distance  The distance between the two arrays.
			 *
			 * \return True if success.
			 */
			bool SearchNeighbour(const Scalar * query, int * indice, DistanceType * distance)
			{
				if (_index.get() != NULL)  {
					int * indicePTR = indice;
					DistanceType * distancePTR = distance;
					flann::Matrix<Scalar> queries((Scalar*)query, 1, _dimension);

					flann::Matrix<int> indices(indicePTR, 1, 1);
					flann::Matrix<DistanceType> dists(distancePTR, 1, 1);
					// do a knn search, using 128 checks
					(*_index).knnSearch(queries, indices, dists, 1, flann::SearchParams(128));

					return true;
				}
				else  {
					return false;
				}
			}


			/**
			   * Search the N nearest Neighbor of the scalar array query.
			   *
			   * \param[in]   query     The query array
			   * \param[in]   query_num   The number of query rows
			   * \param[out]  indice    For each "query" it save the index of the "nearest_neighbor_num"
			   * nearest entry in the dataset (provided in Build).
			   * \param[out]  distance  The distances between the matched arrays.
			   * \param[out]  nearest_neighbor_num        The number of maximal neighbor that will be searched.
			   *
			   * \return True if success.
			   */
			bool SearchNeighbours(const Scalar * query, int query_num,
				std::vector<int> * vec_indice,
				std::vector<DistanceType> * vec_distance,
				size_t nearest_neighbor_num)
			{
				if (_index.get() != NULL)  {
					//-- Check if resultIndices is allocated
					vec_indice->resize(query_num * nearest_neighbor_num);
					vec_distance->resize(query_num * nearest_neighbor_num);

					int * indicePTR = &((*vec_indice)[0]);
					DistanceType * distancePTR = &(*vec_distance)[0];
					flann::Matrix<Scalar> queries((Scalar*)query, query_num, _dimension);

					flann::Matrix<int> indices(indicePTR, query_num, nearest_neighbor_num);
					flann::Matrix<DistanceType> dists(distancePTR, query_num, nearest_neighbor_num);
					// do a knn search, using 128 checks
					(*_index).knnSearch(queries, indices, dists, nearest_neighbor_num, flann::SearchParams(128));
					return true;
				}
				else  {
					return false;
				}
			}

		private:

			std::shared_ptr< flann::Matrix<Scalar> > _datasetM;
			std::shared_ptr< flann::Index<Metric> > _index;
			size_t _dimension;
		};

	} // namespace feature
} // namespace mvg

#endif // MVG_FEATURE_MATCHER_KDTREE_FLANN_H_
