/*************************************************************************
 * 文件： test.cpp
 * 时间： 2015/04/29 15:09
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： exif测试
 *************************************************************************/
#include "mvg/image/exif_simple.h"
#include "mvg/utils/cmd_line.h"

using namespace mvg::utils;

int main(int argc, char *argv[])
{
	CmdLine cmd;
	std::string input_image;
	cmd.add(make_option('i', input_image, "imagefile"));
	try{
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string &s){
		std::cerr << "Usage: " << argv[0] << ' '
			<< "[-i|--imagefile path] "
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "You called: " << std::endl
		<< argv[0] << std::endl
		<< "--imagefile " << input_image << std::endl;

	mvg::image::EXIFSimple exif_sample;
	bool is_parse = exif_sample.open(input_image);
	if (!is_parse)
	{
		std::cerr << "Can't get the exif info!" << std::endl;
	}
	std::cout << "width : " << exif_sample.getWidth() << std::endl;
	std::cout << "height : " << exif_sample.getHeight() << std::endl;
	std::cout << "focal : " << exif_sample.getFocalLength() << std::endl;
	std::cout << "brand : " << exif_sample.getBrand() << std::endl;
	std::cout << "model : " << exif_sample.getModel() << std::endl;
	getchar();
	return EXIT_SUCCESS;
}