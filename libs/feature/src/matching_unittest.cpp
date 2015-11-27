#include "testing.h"
#include <iostream>
#include <vector>

#include "mvg/math/numeric.h"
#include "mvg/feature/matcher_brute_force.h"
#include "mvg/feature/matcher_kdtree_flann.h"

using namespace std;

using namespace mvg::feature;

TEST(Matching, ArrayMatcherBruteForce_Simple_Dim1)
{
  float array[] = {0, 1, 2, 3, 4};
  ArrayMatcherBruteForce<float> matcher;
  EXPECT_TRUE( matcher.Build(array, 5, 1) );

  float query[] = {2};
  int nIndice = -1;
  float fDistance = -1.0f;
  EXPECT_TRUE( matcher.SearchNeighbour( query, &nIndice, &fDistance) );

  EXPECT_EQ( 2, nIndice); // index of the found nearest neighbor
  EXPECT_NEAR( 0.0f, fDistance, 1e-8); //distance
}

TEST(Matching, ArrayMatcherBruteForce_NN)
{
  float array[] = {0, 1, 2, 5, 6};
  // no 3, because it involve the same dist as 1,1
  ArrayMatcherBruteForce<float> matcher;
  EXPECT_TRUE( matcher.Build(array, 5, 1) );

  float query[] = {2};
  vector<int> vec_nIndice;
  vector<float> vec_fDistance;
  EXPECT_TRUE( matcher.SearchNeighbours(query,1, &vec_nIndice, &vec_fDistance, 5) );

  EXPECT_EQ( 5, vec_nIndice.size());
  EXPECT_EQ( 5, vec_fDistance.size());

  // Check distances:
  /*EXPECT_NEAR( vec_fDistance[0], Square(2.0f-2.0f), 1e-6);
  EXPECT_NEAR( vec_fDistance[1], Square(1.0f-2.0f), 1e-6);
  EXPECT_NEAR( vec_fDistance[2], Square(0.0f-2.0f), 1e-6);
  EXPECT_NEAR( vec_fDistance[3], Square(5.0f-2.0f), 1e-6);
  EXPECT_NEAR( vec_fDistance[4], Square(6.0f-2.0f), 1e-6);*/

  // Check indexes:
  EXPECT_EQ(2, vec_nIndice[0]);
  EXPECT_EQ(1, vec_nIndice[1]);
  EXPECT_EQ(0, vec_nIndice[2]);
  EXPECT_EQ(3, vec_nIndice[3]);
  EXPECT_EQ(4, vec_nIndice[4]);
}

TEST(Matching, ArrayMatcherBruteForce_Simple_Dim4)
{
  float array[] = {
    0, 1, 2, 3,
    4, 5, 6, 7,
    8, 9, 10, 11};
  ArrayMatcherBruteForce<float> matcher;
  EXPECT_TRUE( matcher.Build(array, 3, 4) );

  float query[] = {4, 5, 6, 7};
  int nIndice = -1;
  float fDistance = -1.0f;
  EXPECT_TRUE( matcher.SearchNeighbour( query, &nIndice, &fDistance) );

  EXPECT_EQ( 1, nIndice); // index of the found nearest neighbor
  EXPECT_NEAR( 0.0f, fDistance, 1e-8); //distance
}

TEST(Matching, ArrayMatcher_Kdtree_Flann_Simple__NN)
{
  float array[] = {0, 1, 2, 5, 6};
  // no 3, because it involve the same dist as 1,1

  ArrayMatcherKdtreeFlann<float> matcher;
  EXPECT_TRUE( matcher.Build(array, 5, 1) );

  float query[] = {2};
  vector<int> vec_nIndice;
  vector<float> vec_fDistance;
  int nearest_neighbor_num = 5;
  EXPECT_TRUE( matcher.SearchNeighbours(query, 1, &vec_nIndice, &vec_fDistance, nearest_neighbor_num) );

  EXPECT_EQ( 5, vec_nIndice.size());
  EXPECT_EQ( 5, vec_fDistance.size());

  // Check distances:
 /* EXPECT_NEAR( vec_fDistance[0], Square(2.0f-2.0f), 1e-6);
  EXPECT_NEAR( vec_fDistance[1], Square(1.0f-2.0f), 1e-6);
  EXPECT_NEAR( vec_fDistance[2], Square(0.0f-2.0f), 1e-6);
  EXPECT_NEAR( vec_fDistance[3], Square(5.0f-2.0f), 1e-6);
  EXPECT_NEAR( vec_fDistance[4], Square(6.0f-2.0f), 1e-6);*/

  // Check indexes:
  EXPECT_EQ(2, vec_nIndice[0]);
  EXPECT_EQ(1, vec_nIndice[1]);
  EXPECT_EQ(0, vec_nIndice[2]);
  EXPECT_EQ(3, vec_nIndice[3]);
  EXPECT_EQ(4, vec_nIndice[4]);
}

//-- Test LIMIT case (empty arrays)
//TODO:这边注释了，进行单元测试
//TEST(Matching, ArrayMatcherBruteForce_Simple_EmptyArrays)
//{
//  std::vector<float> array;
//  ArrayMatcherBruteForce<float> matcher;
//  EXPECT_FALSE( matcher.Build(&array[0], 0, 4) );
//
//  int nIndice = -1;
//  float fDistance = -1.0f;
//  EXPECT_FALSE( matcher.SearchNeighbour( &array[0], &nIndice, &fDistance) );
//}

//TEST(Matching, ArrayMatcher_Kdtree_Flann_Simple_EmptyArrays)
//{
//  std::vector<float> array;
//  ArrayMatcherKdtreeFlann<float> matcher;
//  EXPECT_FALSE( matcher.Build(&array[0], 0, 4) );
//
//  int nIndice = -1;
//  float fDistance = -1.0f;
//  EXPECT_FALSE( matcher.SearchNeighbour( &array[0], &nIndice, &fDistance) );
//}
