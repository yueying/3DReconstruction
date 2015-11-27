#include "mvg/feature/estimator_line_kernel.h"
#include "testing.h"
#include <vector>
using namespace mvg::math;
using namespace mvg::feature;

// 测试线性模型计算线性模型系数
TEST(LineFitter, ItWorks) {

  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;
  std::vector<Vec2> models;
  LineKernel kernel(xy);
  std::vector<size_t> samples;
  for (size_t i = 0; i < xy.cols(); ++i) {
    samples.push_back(i);
  }
  kernel.Fit(samples, &models);
  EXPECT_EQ(1, models.size());
  EXPECT_NEAR(2.0, models[0][1], 1e-9);
  EXPECT_NEAR(1.0, models[0][0], 1e-9);
}

