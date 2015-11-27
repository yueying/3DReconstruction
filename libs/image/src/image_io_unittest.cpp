#include <cstdio>
#include <iostream>
#include <string>

#include "mvg/image/image.h"
#include "testing.h"

using namespace mvg::image;
using std::string;

namespace mvg {
	namespace utils {
		extern std::string MVG_GLOBAL_SRC_DIR;
	}
}
using namespace mvg::utils;

TEST(ReadJpg, Jpg_Color) {
  Image<RGBColor> image;
  string jpg_filename = MVG_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_color.jpg";
  EXPECT_TRUE(readImage(jpg_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(3, image.Channels());
  EXPECT_EQ(image(0,0), RGBColor(255, 125, 11));
  EXPECT_EQ(image(0,1), RGBColor( 20, 127, 255));
}

TEST(ReadJpg, Jpg_Monochrome) {
  Image<unsigned char> image;
  string jpg_filename = MVG_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_monochrome.jpg";
  EXPECT_TRUE(readImage(jpg_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Channels());
  EXPECT_EQ(image(0,0), (unsigned char)255);
  EXPECT_EQ(image(0,1), (unsigned char)0);
}

TEST(ReadPng, Png_Color) {
  Image<RGBAColor> image;
  string png_filename = MVG_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_color.png";
  EXPECT_TRUE(readImage(png_filename.c_str(), &image));
  // Channels is 4 (RGBA by default)
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(4, image.Channels());
  EXPECT_EQ(image(0,0), RGBAColor(255, 125, 10, 255));
  EXPECT_EQ(image(0,1), RGBAColor( 20, 127, 255,255));
}

TEST(ReadPng, Png_Monochrome) {
  Image<unsigned char> image;
  string png_filename = MVG_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_monochrome.png";
  EXPECT_TRUE(readImage(png_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Channels());
  EXPECT_EQ(image(0,0), (unsigned char)255);
  EXPECT_EQ(image(0,1), (unsigned char)0);
}

TEST(GetFormat, filenames) {
  EXPECT_EQ(GetFormat("something.jpg"), mvg::image::Jpg);
  EXPECT_EQ(GetFormat("something.png"), mvg::image::Png);
  EXPECT_EQ(GetFormat("something.pnm"), mvg::image::Pnm);
  EXPECT_EQ(GetFormat("/some/thing.JpG"), mvg::image::Jpg);
  EXPECT_EQ(GetFormat("/some/thing.pNG"), mvg::image::Png);
  EXPECT_EQ(GetFormat("some/thing.PNm"), mvg::image::Pnm);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.JPG"), mvg::image::Jpg);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.PNG"), mvg::image::Png);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.PNM"), mvg::image::Pnm);
}

TEST(ImageIOTest, Png_Out) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  string out_filename = ("test_write_png.png");
  EXPECT_TRUE(writeImage(out_filename.c_str(), image));

  Image<unsigned char> read_image;
  EXPECT_TRUE(readImage(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(out_filename.c_str());
}

TEST(ImageIOTest, Png_Out_Color) {
  Image<RGBColor> image(1,2);
  image(0,0) = RGBColor(255,127,0);
  image(1,0) = RGBColor(0,127,255);
  string out_filename = ("test_write_png_color.png");
  EXPECT_TRUE(writeImage(out_filename.c_str(), image));

  Image<RGBColor> read_image;
  EXPECT_TRUE(readImage(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(out_filename.c_str());
}

//TEST(ImageIOTest, InvalidFiles) {
//  Image<unsigned char> image;
//  string filename = MVG_GLOBAL_SRC_DIR + "/donotexist.jpg";
//  EXPECT_FALSE(readImage(filename.c_str(), &image));
//  EXPECT_FALSE(readImage("hopefully_unexisting_file", &image));
//  remove(filename.c_str());
//}

TEST(ImageIOTest, Jpg) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  string filename = ("test_write_jpg.jpg");
  EXPECT_TRUE(WriteJpg(filename.c_str(), image, 100));

  Image<unsigned char> read_image;
  EXPECT_TRUE(readImage(filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(filename.c_str());
}

TEST(ReadPnm, Pgm) {
  Image<unsigned char> image;
  string pgm_filename = MVG_GLOBAL_SRC_DIR + "/data/image_test/two_pixels.pgm";
  EXPECT_TRUE(readImage(pgm_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Channels());
  EXPECT_EQ(image(0,0), (unsigned char)255);
  EXPECT_EQ(image(0,1), (unsigned char)0);
}

TEST(ReadPnm, PgmComments) {
  Image<unsigned char> image;
  string pgm_filename = MVG_GLOBAL_SRC_DIR + "/data/image_test/two_pixels_gray.pgm";
  EXPECT_TRUE(readImage(pgm_filename.c_str(), &image));
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
  EXPECT_TRUE(writeImage(out_filename.c_str(),image));

  Image<unsigned char> read_image;
  EXPECT_TRUE(readImage(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(out_filename.c_str());
}

TEST(ReadPnm, Ppm) {
  Image<RGBColor> image;
  string ppm_filename = MVG_GLOBAL_SRC_DIR + "/data/image_test/two_pixels.ppm";
  EXPECT_TRUE(readImage(ppm_filename.c_str(), &image));
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
  EXPECT_TRUE(writeImage(out_filename.c_str(), image));

  Image<RGBColor> read_image;
  EXPECT_TRUE(readImage(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
  remove(out_filename.c_str());
}

