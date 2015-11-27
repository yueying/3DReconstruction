#include "mvg/feature/indexed_match.h"
#include "mvg/feature/indexed_match_utils.h"
#include "mvg/image/image.h"
#include "mvg/feature/features.h"

#include "mvg/feature/image_list_io_helper.h"
#include "mvg/utils/cmd_line.h"
#include "mvg/utils/file_system.h"
#include "mvg/utils/progress.h"
#include "mvg/utils/svg_drawer.h"

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <map>

using namespace mvg::utils;
using namespace mvg::feature;

int main(int argc, char ** argv)
{
	CmdLine cmd;

	std::string image_dir;
	std::string matches_dir;
	std::string out_dir = "";

	cmd.add(make_option('i', image_dir, "imadir"));
	cmd.add(make_option('d', matches_dir, "matchdir"));
	cmd.add(make_option('o', out_dir, "outdir"));

	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Export pairwise matches.\nUsage: " << argv[0] << "\n"
			<< "[-i|--imadir path]\n"
			<< "[-d|--matchdir path]\n"
			<< "[-o|--outdir path]\n"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	if (out_dir.empty())  {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}


	//---------------------------------------
	// Read images names
	//---------------------------------------

	std::vector<mvg::feature::CameraInfo> vec_camera_info;
	std::vector<mvg::feature::IntrinsicCameraInfo> vec_cameras_intrinsic;
	if (!mvg::feature::loadImageList(
		mvg::utils::create_filespec(matches_dir, "lists", "txt"),
		vec_camera_info,
		vec_cameras_intrinsic
		))
	{
		std::cerr << "\nEmpty image list." << std::endl;
		return false;
	}


	// ------------
	// For each image, export visually the keypoints
	// ------------

	mvg::utils::folder_create(out_dir);
	std::cout << "\n Export extracted keypoints for all images" << std::endl;
	ControlProgressDisplay my_progress_bar(vec_camera_info.size());
	for (std::vector<mvg::feature::CameraInfo>::const_iterator iterFilename = vec_camera_info.begin();
		iterFilename != vec_camera_info.end();
		++iterFilename, ++my_progress_bar)
	{
		const size_t I = std::distance(
			(std::vector<mvg::feature::CameraInfo>::const_iterator)vec_camera_info.begin(),
			iterFilename);

		const std::pair<size_t, size_t>
			dimImage = std::make_pair(vec_cameras_intrinsic[iterFilename->intrinsic_id].width,
			vec_cameras_intrinsic[iterFilename->intrinsic_id].height);

		SvgDrawer svg_stream(dimImage.first, dimImage.second);
		svg_stream.drawImage(mvg::utils::create_filespec(image_dir, iterFilename->image_name),
			dimImage.first,
			dimImage.second);

		// Load the features from the feature file
		std::vector<ScalePointFeature> vec_feat;
		LoadFeatsFromFile(
			mvg::utils::create_filespec(matches_dir, mvg::utils::basename_part(iterFilename->image_name), ".feat"),
			vec_feat);

		//-- Draw features
		for (size_t i = 0; i < vec_feat.size(); ++i)  {
			const ScalePointFeature & feature = vec_feat[i];
			svg_stream.drawCircle(feature.x(), feature.y(), feature.scale(),
				SvgStyle().stroke("yellow", 2.0));
		}

		// Write the SVG file
		std::ostringstream os;
		os << mvg::utils::folder_append_separator(out_dir)
			<< mvg::utils::basename_part(iterFilename->image_name)
			<< "_" << vec_feat.size() << "_.svg";
		ofstream svg_file(os.str().c_str());
		svg_file << svg_stream.closeSvgFile().str();
		svg_file.close();
	}
	return EXIT_SUCCESS;
}
