#ifndef FBLIB_FEATURE_MATCHER_KDTREE_FLANN_H_
#define FBLIB_FEATURE_MATCHER_KDTREE_FLANN_H_


#include "fblib/feature/matching_interface.h"
#include "flann/flann.h"
#include <memory>

namespace fblib {
namespace feature  {

/// Implement ArrayMatcher as a FLANN KDtree matcher.
// http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN
// David G. Lowe and Marius Muja
//
// By default use squared L2 metric (flann::L2<Scalar>)
// sqrt is monotonic so for performance reason we do not compute it.

template < typename Scalar = float, typename  Metric = flann::L2<Scalar> >
class ArrayMatcherKdtreeFlann : public ArrayMatcher<Scalar, Metric>
{
  public:
  typedef typename Metric::ResultType DistanceType;

  ArrayMatcherKdtreeFlann() {}

  virtual ~ArrayMatcherKdtreeFlann()  {
    _datasetM.reset();
    _index.reset();
  }

  /**
   * Build the matching structure
   *
   * \param[in] dataset   Input data.
   * \param[in] rows_num    The number of component.
   * \param[in] dimension Length of the data contained in the each
   *  row of the dataset.
   *
   * \return True if success.
   */
  bool Build( const Scalar * dataset, int rows_num, int dimension)  {

    if (rows_num > 0)
    {
      _dimension = dimension;
      //-- Build Flann Matrix container (map to already allocated memory)
      _datasetM = std::auto_ptr< flann::Matrix<Scalar> >(
          new flann::Matrix<Scalar>((Scalar*)dataset, rows_num, dimension));

      //-- Build FLANN index
	  _index = std::auto_ptr< flann::Index<Metric> >(
          new flann::Index<Metric> (*_datasetM, flann::KDTreeIndexParams(4)) );
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
  bool SearchNeighbour( const Scalar * query, int * indice, DistanceType * distance)
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
  bool SearchNeighbours( const Scalar * query, int query_num,
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

  private :

  std::auto_ptr< flann::Matrix<Scalar> > _datasetM;
  std::auto_ptr< flann::Index<Metric> > _index;
  size_t _dimension;
};

} // namespace feature
} // namespace fblib



#endif // FBLIB_FEATURE_MATCHER_KDTREE_FLANN_H_
