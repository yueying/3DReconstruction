#include "mvg/image/exif_simple.h"
#include "testing.h"

#include <iostream>
#include <memory>

using namespace std;
using namespace mvg::image;

namespace mvg {
	namespace utils {
		extern std::string MVG_GLOBAL_SRC_DIR;
	}
}
using namespace mvg::utils;
namespace {

	TEST(Image, Exif_IO_easyexif_ReadData)
	{
		string exif_filename = MVG_GLOBAL_SRC_DIR + "/data/image_test/exif.jpg";
		EXIFSimple exif_io(exif_filename);

		EXPECT_TRUE(exif_io.doesHaveExifInfo());

		EXPECT_EQ("EASTMAN KODAK COMPANY", exif_io.getBrand());
		EXPECT_EQ("KODAK Z612 ZOOM DIGITAL CAMERA", exif_io.getModel());

		EXPECT_EQ(2832, exif_io.getWidth());
		EXPECT_EQ(2128, exif_io.getHeight());
		EXPECT_NEAR(5.85, exif_io.getFocalLength(), 1e-2);

		EXPECT_EQ("", exif_io.getLensModel());
	}
}

