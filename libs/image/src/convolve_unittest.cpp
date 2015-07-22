#include <iostream>

#include "fblib/image/convolve.h"
#include "fblib/image/image.h"
#include "fblib/math/numeric.h"
#include "testing.h"

using namespace fblib::image;
using namespace fblib::math;

TEST(Convolve, ComputeGaussianKernel) {
  Vec kernel, derivative;
  ComputeGaussianKernel(1, &kernel, &derivative);
  EXPECT_EQ(7, kernel.size());
   //TODO(fengbing): 添加一个更全面点的测试
}

TEST(Convolve, ConvolveGaussian) {
  Image<float> im(10, 10);
  im.fill(1);
  Image<float> blured;
  ConvolveGaussian(im, 3, &blured);
  EXPECT_NEAR(im(5, 5), 1, 1e-7);
}

TEST(Convolve, BoxFilterHorizontal) {
  Image<float> im(10, 10), convolved, filtered;
  im.fill(1);
  BoxFilterHorizontal(im, 3, &filtered);
  Vec kernel(3);
  kernel.setConstant(1.);
  ConvolveHorizontal(im, kernel, &convolved);
  EXPECT_EQ(filtered(5, 5), 3);
  EXPECT_TRUE(filtered == convolved);
}

TEST(Convolve, BoxFilter) {
  Image<float> image(5, 5), filtered;
  // 单一的1在5x5的图像中滤波之后变成3x3方阵.
  image.fill(0);
  image(2, 2) = 1.0;
  BoxFilter(image, 3, &filtered);
  for (int j = 0; j < 5; j++) {
    for (int i = 0; i < 5; i++) {
      if (i == 0 || i == 4 || j == 0 || j == 4) {
        EXPECT_EQ(0.0, filtered(j, i));
      } else {
        EXPECT_EQ(1.0, filtered(j, i));
      }
    }
  }
}

TEST(Convolve, BlurredImageAndDerivativesChannelsFlat) {
  Image<float> im(10, 10);
  im.fill(1);
  Image <RGBfColor> blurred_and_derivatives;
  BlurredImageAndDerivativesChannels(im, 1.0, &blurred_and_derivatives);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(0), 1.0, 1e-7);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(1), 0.0, 1e-7);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(2), 0.0, 1e-7);
}

TEST(Convolve, BlurredImageAndDerivativesChannelsHorizontalSlope) {
  Image<float> image(10, 10);
  for (int j = 0; j < 10; ++j) {
    for (int i = 0; i < 10; ++i) {
      image(j, i) = 2*i;
    }
  }
  Image <RGBfColor> blurred_and_derivatives;
  BlurredImageAndDerivativesChannels(image, 0.9, &blurred_and_derivatives);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(0), 10.0, 1e-7);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(1),  2.0, 1e-7);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(2),  0.0, 1e-7);
}

TEST(Convolve, BlurredImageAndDerivativesChannelsVerticalSlope) {
  Image<float> image(10, 10);
  for (int j = 0; j < 10; ++j) {
    for (int i = 0; i < 10; ++i) {
      image(j, i) = 2*j;
    }
  }
  Image <RGBfColor> blurred_and_derivatives;
  BlurredImageAndDerivativesChannels(image, 0.9, &blurred_and_derivatives);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(0), 10.0, 1e-7);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(1),  0.0, 1e-7);
  EXPECT_NEAR(blurred_and_derivatives(5, 5)(2),  2.0, 1e-7);
}
