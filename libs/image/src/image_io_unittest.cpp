#include <cstdio>
#include <iostream>
#include <string>

#include "fblib/image/image.h"
#include "testing.h"

using namespace fblib::image;
using std::string;

namespace fblib {
	namespace utils {
		extern std::string FBLIB_GLOBAL_SRC_DIR;
	}
}
using namespace fblib::utils;

TEST(ReadJpg, Jpg_Color) {
  Image<RGBColor> image;
  string jpg_filename = FBLIB_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_color.jpg";
  EXPECT_TRUE(ReadImage(jpg_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(3, image.Channels());
  EXPECT_EQ(image(0,0), RGBColor(255, 125, 11));
  EXPECT_EQ(image(0,1), RGBColor( 20, 127, 255));
}

TEST(ReadJpg, Jpg_Monochrome) {
  Image<unsigned char> image;
  string jpg_filename = FBLIB_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_monochrome.jpg";
  EXPECT_TRUE(ReadImage(jpg_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Channels());
  EXPECT_EQ(image(0,0), (unsigned char)255);
  EXPECT_EQ(image(0,1), (unsigned char)0);
}

TEST(ReadPng, Png_Color) {
  Image<RGBAColor> image;
  string png_filename = FBLIB_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_color.png";
  EXPECT_TRUE(ReadImage(png_filename.c_str(), &image));
  // Channels is 4 (RGBA by default)
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(4, image.Channels());
  EXPECT_EQ(image(0,0), RGBAColor(255, 125, 10, 255));
  EXPECT_EQ(image(0,1), RGBAColor( 20, 127, 255,255));
}

TEST(ReadPng, Png_Monochrome) {
  Image<unsigned char> image;
  string png_filename = FBLIB_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_monochrome.png";
  EXPECT_TRUE(ReadImage(png_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Channels());
  EXPECT_EQ(image(0,0), (unsigned char)255);
  EXPECT_EQ(image(0,1), (unsigned char)0);
}

TEST(GetFormat, filenames) {
  EXPECT_EQ(GetFormat("something.jpg"), fblib::image::Jpg);
  EXPECT_EQ(GetFormat("something.png"), fblib::image::Png);
  EXPECT_EQ(GetFormat("something.pnm"), fblib::image::Pnm);
  EXPECT_EQ(GetFormat("/some/thing.JpG"), fblib::image::Jpg);
  EXPECT_EQ(GetFormat("/some/thing.pNG"), fblib::image::Png);
  EXPECT_EQ(GetFormat("some/thing.PNm"), fblib::image::Pnm);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.JPG"), fblib::image::Jpg);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.PNG"), fblib::image::Png);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.PNM"), fblib::image::Pnm);
}

TEST(ImageIOTest, Png_Out) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  string out_filename = ("test_write_png.png");
  EXPECT_TRUE(WriteImage(out_filename.c_str(), image));

  Image<unsigned char> read_image;
  EXPECT_TRUE(ReadImage(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(out_filename.c_str());
}

TEST(ImageIOTest, Png_Out_Color) {
  Image<RGBColor> image(1,2);
  image(0,0) = RGBColor(255,127,0);
  image(1,0) = RGBColor(0,127,255);
  string out_filename = ("test_write_png_color.png");
  EXPECT_TRUE(WriteImage(out_filename.c_str(), image));

  Image<RGBColor> read_image;
  EXPECT_TRUE(ReadImage(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(out_filename.c_str());
}

//TEST(ImageIOTest, InvalidFiles) {
//  Image<unsigned char> image;
//  string filename = FBLIB_GLOBAL_SRC_DIR + "/donotexist.jpg";
//  EXPECT_FALSE(ReadImage(filename.c_str(), &image));
//  EXPECT_FALSE(ReadImage("hopefully_unexisting_file", &image));
//  remove(filename.c_str());
//}

TEST(ImageIOTest, Jpg) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  string filename = ("test_write_jpg.jpg");
  EXPECT_TRUE(WriteJpg(filename.c_str(), image, 100));

  Image<unsigned char> read_image;
  EXPECT_TRUE(ReadImage(filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(filename.c_str());
}

TEST(ReadPnm, Pgm) {
  Image<unsigned char> image;
  string pgm_filename = FBLIB_GLOBAL_SRC_DIR + "/data/image_test/two_pixels.pgm";
  EXPECT_TRUE(ReadImage(pgm_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Channels());
  EXPECT_EQ(image(0,0), (unsigned char)255);
  EXPECT_EQ(image(0,1), (unsigned char)0);
}

TEST(ReadPnm, PgmComments) {
  Image<unsigned char> image;
  string pgm_filename = FBLIB_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_gray.pgm";
  EXPECT_TRUE(ReadImage(pgm_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Channels());
  EXPECT_EQ(image(0,0), (unsigned char)255);
  EXPECT_EQ(image(0,1), (unsigned char)0);
}


TEST(ImageIOTest, Pgm) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  string out_filename = "test_write_pnm.pgm";
  EXPECT_TRUE(WriteImage(out_filename.c_str(),image));

  Image<unsigned char> read_image;
  EXPECT_TRUE(ReadImage(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(out_filename.c_str());
}

TEST(ReadPnm, Ppm) {
  Image<RGBColor> image;
  string ppm_filename = FBLIB_GLOBAL_SRC_DIR + "/data/image_test/two_pixels.ppm";
  EXPECT_TRUE(ReadImage(ppm_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(3, image.Channels());
  EXPECT_EQ(image(0,0), RGBColor( (unsigned char)255));
  EXPECT_EQ(image(0,1), RGBColor( (unsigned char)0));
}

TEST(ImageIOTest, Ppm) {
  Image<RGBColor> image(1,2);
  image(0,0) = RGBColor((unsigned char)255);
  image(1,0) = RGBColor((unsigned char)0);
  string out_filename = "test_write_pnm.ppm";
  EXPECT_TRUE(WriteImage(out_filename.c_str(), image));

  Image<RGBColor> read_image;
  EXPECT_TRUE(ReadImage(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(out_filename.c_str());
}

