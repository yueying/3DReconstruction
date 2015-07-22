#include "testing.h"
#include "fblib/feature/indexed_match.h"

using namespace fblib::feature;

TEST(IndexedMatch, DuplicateRemoval_NoRemoval)
{
  std::vector<IndexedMatch> vec_indexed_match;

  vec_indexed_match.push_back(IndexedMatch(2,3)); // 0
  vec_indexed_match.push_back(IndexedMatch(0,1)); // 1

  // 检查，没有重复项，返回false
  EXPECT_FALSE(IndexedMatch::getDeduplicated(vec_indexed_match));

  // 调用上述之后，自动排序
  EXPECT_EQ(IndexedMatch(0,1), vec_indexed_match[0]);
  EXPECT_EQ(IndexedMatch(2,3), vec_indexed_match[1]);
}

TEST(IndexedMatch, DuplicateRemoval_Simple)
{
  std::vector<IndexedMatch> vec_indexed_match;

  vec_indexed_match.push_back(IndexedMatch(0,1)); // 0
  vec_indexed_match.push_back(IndexedMatch(0,1)); // 1: error with addition 0

  vec_indexed_match.push_back(IndexedMatch(1,2)); // 2
  vec_indexed_match.push_back(IndexedMatch(1,2)); // 3: error with addition 2

  EXPECT_TRUE(IndexedMatch::getDeduplicated(vec_indexed_match));
  // 还剩两个匹配(line 0 and 2)
  EXPECT_EQ(2, vec_indexed_match.size());
}

TEST(IndexedMatch, DuplicateRemoval)
{
  std::vector<IndexedMatch> vec_indexed_match;

  vec_indexed_match.push_back(IndexedMatch(0,1)); // 0
  vec_indexed_match.push_back(IndexedMatch(0,2)); // 1: error with addition 0

  vec_indexed_match.push_back(IndexedMatch(1,1)); // 2: error with addition 1

  vec_indexed_match.push_back(IndexedMatch(2,3)); // 3
  vec_indexed_match.push_back(IndexedMatch(3,3)); // 4: error with addition 3

  EXPECT_TRUE(IndexedMatch::getDeduplicated(vec_indexed_match));
  // 还剩两个匹配 (line 0 and 3)
  EXPECT_EQ(2, vec_indexed_match.size());

  EXPECT_EQ(IndexedMatch(0,1), vec_indexed_match[0]);
  EXPECT_EQ(IndexedMatch(2,3), vec_indexed_match[1]);
}
