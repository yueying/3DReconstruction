#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "fblib/image/exif_simple.h"
#include "fblib/image/image.h"
#include "fblib/image/image_io.h"
#include "fblib/utils/parse_database.h"

#include "fblib/utils/cmd_line.h"
#include "fblib/utils/file_system.h"

#ifdef max
#undef max
#endif // max

using namespace fblib::image;
using namespace fblib::utils;

int main(int argc, char **argv)
{
	CmdLine cmd;

	std::string image_dir;
	std::string file_database = "";
	std::string output_dir = "";
	double focal_pix_per_mm = -1.0;

	cmd.add(make_option('i', image_dir, "imageDirectory"));
	cmd.add(make_option('d', file_database, "sensorWidthDatabase"));
	cmd.add(make_option('o', output_dir, "outputDirectory"));
	cmd.add(make_option('f', focal_pix_per_mm, "focal"));

	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Usage: " << argv[0] << '\n'
			<< "[-i|--imageDirectory]\n"
			<< "[-d|--sensorWidthDatabase]\n"
			<< "[-o|--outputDirectory]\n"
			<< "[-f|--focal] (pixels)\n"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << " You called : " << std::endl
		<< argv[0] << std::endl
		<< "--imageDirectory " << image_dir << std::endl
		<< "--sensorWidthDatabase " << file_database << std::endl
		<< "--outputDirectory " << output_dir << std::endl
		<< "--focal " << focal_pix_per_mm << std::endl;

	if (!fblib::utils::folder_exists(image_dir))
	{
		std::cerr << "\nThe input directory doesn't exist" << std::endl;
		return EXIT_FAILURE;
	}

	if (output_dir.empty())
	{
		std::cerr << "\nInvalid output directory" << std::endl;
		return EXIT_FAILURE;
	}

	if (!fblib::utils::folder_exists(output_dir))
	{
		if (!fblib::utils::folder_create(output_dir))
		{
			std::cerr << "\nCannot create output directory" << std::endl;
			return EXIT_FAILURE;
		}
	}

	std::vector<std::string> vec_image = fblib::utils::folder_files(image_dir);
	// 写入新的文件
	std::ofstream list_txt(fblib::utils::create_filespec(output_dir,
		"lists.txt").c_str());
	if (list_txt)
	{
		std::sort(vec_image.begin(), vec_image.end());
		for (std::vector<std::string>::const_iterator iter_image = vec_image.begin();
			iter_image != vec_image.end();
			iter_image++)
		{
			// 读取数据获得图像高度宽度，和相机焦距
			std::string image_filename = fblib::utils::create_filespec(image_dir, *iter_image);

			size_t width = -1;
			size_t height = -1;

			std::shared_ptr<EXIFSimple> exif_reader(new EXIFSimple());
			exif_reader->open(image_filename);

			//分情况讨论，考虑focal是否提供

			std::ostringstream os;
			//如果图像没有含有图片头文件信息
			if (!exif_reader->doesHaveExifInfo() || focal_pix_per_mm != -1)
			{
				Image<unsigned char> image;
				if (readImage(image_filename.c_str(), &image))  {
					width = image.Width();
					height = image.Height();
				}
				else
				{
					Image<RGBColor> imageRGB;
					if (readImage(image_filename.c_str(), &imageRGB)) {
						width = imageRGB.Width();
						height = imageRGB.Height();
					}
					else
					{
						Image<RGBAColor> imageRGBA;
						if (readImage(image_filename.c_str(), &imageRGBA))  {
							width = imageRGBA.Width();
							height = imageRGBA.Height();
						}
						else
							continue; // 还没有考虑其他类型的图像，先过
					}
				}
				if (focal_pix_per_mm == -1)
					os << *iter_image << ";" << width << ";" << height << std::endl;
				else
					os << *iter_image << ";" << width << ";" << height << ";"
					<< focal_pix_per_mm << ";" << 0 << ";" << width / 2.0 << ";"
					<< 0 << ";" << focal_pix_per_mm << ";" << height / 2.0 << ";"
					<< 0 << ";" << 0 << ";" << 1 << std::endl;

			}
			else // 图像包含头文件信息
			{
				double focal = focal_pix_per_mm;
				width = exif_reader->getWidth();
				height = exif_reader->getHeight();
				std::string cam_name = exif_reader->getBrand();
				std::string cam_model = exif_reader->getModel();

				std::vector<Datasheet> vec_database;
				Datasheet datasheet;
				if (parseDatabase(file_database, vec_database))
				{
					if (getInfo(cam_name, cam_model, vec_database, datasheet))
					{
						// 相机型号在数据库中发现，我们可以计算近似的focal length
						double ccdw = datasheet._sensorSize;
						focal = std::max(width, height) * exif_reader->getFocalLength() / ccdw;
						os << *iter_image << ";" << width << ";" << height << ";" << focal << ";" << cam_name << ";" << cam_model << std::endl;
					}
					else
					{
						std::cout << "Camera \"" << cam_name << "\" model \"" << cam_model << "\" doesn't exist in the database" << std::endl;
						os << *iter_image << ";" << width << ";" << height << ";" << cam_name << ";" << cam_model << std::endl;
					}
				}
				else
				{
					std::cout << "Sensor width database \"" << file_database << "\" doesn't exist." << std::endl;
					std::cout << "Please consider add your camera model in the database." << std::endl;
					os << *iter_image << ";" << width << ";" << height << ";" << cam_name << ";" << cam_model << std::endl;
				}
			}
			std::cout << os.str();
			list_txt << os.str();
		}
	}
	list_txt.close();
	return EXIT_SUCCESS;
}