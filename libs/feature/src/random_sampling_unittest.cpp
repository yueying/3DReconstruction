#include "fblib/feature/random_sampling.h"
#include "testing.h"
#include <set>

using namespace fblib::feature;

// 通过验证随机取出所有的数据，无重复的方式来进行验证
TEST(UniformSampleTest, NoRepetions) {

  std::vector<size_t> samples;
  for (size_t total = 1; total < 500; total *= 2) { //数据集大小
    for (size_t num_samples = 1; num_samples <= total; num_samples *= 2) { //给出一致的大小
      UniformSample(num_samples, total, &samples);
      std::set<size_t> myset;
      for (size_t i = 0; i < num_samples; ++i) {
        myset.insert(samples[i]);
      }
	  EXPECT_EQ(num_samples, myset.size());
    }
  }
}

// 通过验证随机取出所有的数据，无重复的方式来进行验证
TEST(RandomSampleTest, NoRepetions) {

  std::vector<size_t> samples;
  for (size_t total = 1; total < 500; total *= 2) { //数据集大小
    for (size_t num_samples = 1; num_samples <= total; num_samples *= 2) { //给出一致的大小
      RandomSample(num_samples, total, &samples);
      std::set<size_t> myset;
      for (size_t i = 0; i < num_samples; ++i) {
        myset.insert(samples[i]);
      }
	  EXPECT_EQ(num_samples, myset.size());
    }
  }
}
