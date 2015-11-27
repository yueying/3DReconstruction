#include "mvg/image/sample.h"
#include "testing.h"

using namespace mvg::image;

TEST(Image, Nearest) {
	Image<unsigned char> image(2, 2);
	image(0, 0) = 0;
	image(0, 1) = 1;
	image(1, 0) = 2;
	image(1, 1) = 3;
	EXPECT_EQ(0, SampleNearest(image, -0.4f, -0.4f));
	EXPECT_EQ(0, SampleNearest(image, 0.4f, 0.4f));
	EXPECT_EQ(3, SampleNearest(image, 0.6f, 0.6f));
	EXPECT_EQ(3, SampleNearest(image, 1.4f, 1.4f));
}

TEST(Image, Linear) {
	Image<float> image(2, 2);
	image(0, 0) = 0;
	image(0, 1) = 1;
	image(1, 0) = 2;
	image(1, 1) = 3;
	EXPECT_EQ(1.5, SampleLinear(image, 0.5, 0.5));
}

TEST(Image, DownsampleBy2) {
	Image<float> image(2, 2);
	image(0, 0) = 0;
	image(0, 1) = 1;
	image(1, 0) = 2;
	image(1, 1) = 3;
	Image<float> resampled_image;
	DownsampleChannelsBy2(image, &resampled_image);
	ASSERT_EQ(1, resampled_image.Height());
	ASSERT_EQ(1, resampled_image.Width());
	ASSERT_EQ(1, resampled_image.Channels());
	EXPECT_FLOAT_EQ(6. / 4., resampled_image(0, 0));
}

TEST(Image, DownsampleBy2MultiChannel) {
	Image<RGBfColor> image(2, 2);
	image(0, 0) = RGBfColor(0, 5, 9);
	image(0, 1) = RGBfColor(1, 6, 10);
	image(1, 0) = RGBfColor(2, 7, 11);
	image(1, 1) = RGBfColor(3, 8, 12);

	Image<RGBfColor> resampled_image;
	DownsampleChannelsBy2(image, &resampled_image);
	ASSERT_EQ(1, resampled_image.Height());
	ASSERT_EQ(1, resampled_image.Width());
	ASSERT_EQ(3, resampled_image.Channels());
	EXPECT_FLOAT_EQ((0 + 1 + 2 + 3) / 4., resampled_image(0, 0)(0));
	EXPECT_FLOAT_EQ((5 + 6 + 7 + 8) / 4., resampled_image(0, 0)(1));
	EXPECT_FLOAT_EQ((9 + 10 + 11 + 12) / 4., resampled_image(0, 0)(2));
}

