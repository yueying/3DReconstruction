#include "testing.h"
#include "fblib/feature/metric.h"
#include "flann/algorithms/dist.h"
#include <iostream>
#include <bitset>
#include <string>

using namespace fblib::feature;

template<typename Metric>
typename Metric::ResultType DistanceT()
{
  typename Metric::ElementType array1[] = {0, 1, 2, 3, 4, 5, 6, 7};
  typename Metric::ElementType array2[] = {7, 6, 5, 4, 3, 2, 1, 0};
  Metric metric;
  return metric(array1, array2, 8);
}

TEST(Metric, SquaredEuclideanDistanceSimple)
{
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceSimple<unsigned char> >());
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceSimple<short> >());
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceSimple<int> >());
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceSimple<float> >());
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceSimple<double> >());
}

TEST(Metric, SquaredEuclideanDistanceVectorized)
{
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceVectorized<unsigned char> >());
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceVectorized<short> >());
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceVectorized<int> >());
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceVectorized<float> >());
  EXPECT_EQ(168, DistanceT<SquaredEuclideanDistanceVectorized<double> >());
}
