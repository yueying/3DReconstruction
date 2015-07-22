#include "testing.h"

#include "fblib/feature/matching_filters.h"
using namespace fblib::feature;

// 测试排序数组的交集
TEST( matching, setIntersection)
{
  int tab0[] = {0, 1, 2, 3, 4, 5, 6, 7};
  int tab1[] = {0, 1, 8, 3, 4, 9, 6, 7};
  std::set<int> vec_0(tab0, tab0 + 8);
  std::set<int> vec_1(tab1, tab1 + 8);

  std::vector<int> vec_intersect;
  IntersectMatches(vec_0.begin(), vec_0.end(),
    vec_1.begin(), vec_1.end(),
    vec_intersect);

  EXPECT_EQ(6, vec_intersect.size());
  EXPECT_EQ(0, vec_intersect[0]);
  EXPECT_EQ(1, vec_intersect[1]);
  EXPECT_EQ(3, vec_intersect[2]);
  EXPECT_EQ(4, vec_intersect[3]);
  EXPECT_EQ(6, vec_intersect[4]);
  EXPECT_EQ(7, vec_intersect[5]);
}
