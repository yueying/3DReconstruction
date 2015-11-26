#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <map>

#include "fblib/feature/indexed_match.h"
#include "fblib/feature/indexed_match_utils.h"
#include "fblib/image/image.h"
#include "fblib/feature/features.h"

#include "fblib/feature/image_list_io_helper.h"
#include "fblib/utils/cmd_line.h"
#include "fblib/utils/file_system.h"
#include "fblib/utils/progress.h"
#include "fblib/utils/svg_drawer.h"

using namespace fblib::utils;
using namespace fblib::feature;

int main(int argc, char ** argv)
{
	CmdLine cmd;

	std::string image_dir;
	std::string matches_dir;
	std::string match_file;
	std::string out_dir = "";

	cmd.add(make_option('i', image_dir, "imadir"));
	cmd.add(make_option('d', matches_dir, "matchdir"));
	cmd.add(make_option('m', match_file, "matchfile"));
	cmd.add(make_option('o', out_dir, "outdir"));

	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Export pairwise matches.\nUsage: " << argv[0] << "\n"
			<< "[-i|--imadir path]\n"
			<< "[-d|--matchdir path]\n"
			<< "[-m|--matchfile filename]\n"
			<< "[-o|--outdir path]\n"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	if (out_dir.empty())  {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}

	// 读取图像文件名
	std::vector<fblib::feature::CameraInfo> vec_camera_info;
	std::vector<fblib::feature::IntrinsicCameraInfo> vec_cameras_intrinsic;
	if (!fblib::feature::loadImageList(
		fblib::utils::create_filespec(matches_dir, "lists", "txt"),
		vec_camera_info,
		vec_cameras_intrinsic
		))
	{
		std::cerr << "\nEmpty image list." << std::endl;
		return false;
	}

	// 从文件中导入匹配索引值
	PairWiseMatches map_matches;
	pairedIndexedMatchImport(match_file, map_matches);

	// 导出匹配对
	fblib::utils::folder_create(out_dir);
	std::cout << "\n Export pairwise matches" << std::endl;
	ControlProgressDisplay my_progress_bar(map_matches.size());
	for (PairWiseMatches::const_iterator iter = map_matches.begin();
		iter != map_matches.end();
		++iter, ++my_progress_bar)
	{
		const size_t I = iter->first.first;
		const size_t J = iter->first.second;

		std::vector<fblib::feature::CameraInfo>::const_iterator caminfoI_it = vec_camera_info.begin() + I;
		std::vector<fblib::feature::CameraInfo>::const_iterator caminfoJ_it = vec_camera_info.begin() + J;

		const std::pair<size_t, size_t>
			dim_image0 = std::make_pair(vec_cameras_intrinsic[caminfoI_it->intrinsic_id].width, vec_cameras_intrinsic[caminfoI_it->intrinsic_id].height),
			dim_image1 = std::make_pair(vec_cameras_intrinsic[caminfoJ_it->intrinsic_id].width, vec_cameras_intrinsic[caminfoJ_it->intrinsic_id].height);

		SvgDrawer svg_stream(dim_image0.first + dim_image1.first, max(dim_image0.second, dim_image1.second));
		svg_stream.drawImage(fblib::utils::create_filespec(image_dir, vec_camera_info[I].image_name),
			dim_image0.first,
			dim_image0.second);
		svg_stream.drawImage(fblib::utils::create_filespec(image_dir, vec_camera_info[J].image_name),
			dim_image1.first,
			dim_image1.second, dim_image0.first);

		const vector<IndexedMatch> &vec_filtered_matches = iter->second;

		if (!vec_filtered_matches.empty()) {
			// 从特征文件中导入特征
			std::vector<ScalePointFeature> vec_featI, vec_featJ;
			LoadFeatsFromFile(
				fblib::utils::create_filespec(matches_dir, fblib::utils::basename_part(vec_camera_info[I].image_name), ".feat"),
				vec_featI);
			LoadFeatsFromFile(
				fblib::utils::create_filespec(matches_dir, fblib::utils::basename_part(vec_camera_info[J].image_name), ".feat"),
				vec_featJ);

			// 在两幅特征之间划线
			for (size_t i = 0; i < vec_filtered_matches.size(); ++i)  {
				const ScalePointFeature & left_img = vec_featI[vec_filtered_matches[i]._i];
				const ScalePointFeature & right_img = vec_featJ[vec_filtered_matches[i]._j];
				svg_stream.drawLine(left_img.x(), left_img.y(),
					right_img.x() + dim_image0.first, right_img.y(), SvgStyle().stroke("green", 2.0));
			}

			// 绘制特征
			for (size_t i = 0; i < vec_filtered_matches.size(); ++i)  {
				const ScalePointFeature & left_img = vec_featI[vec_filtered_matches[i]._i];
				const ScalePointFeature & right_img = vec_featJ[vec_filtered_matches[i]._j];
				svg_stream.drawCircle(left_img.x(), left_img.y(), left_img.scale(),
					SvgStyle().stroke("yellow", 2.0));
				svg_stream.drawCircle(right_img.x() + dim_image0.first, right_img.y(), right_img.scale(),
					SvgStyle().stroke("yellow", 2.0));
			}
		}
		std::ostringstream os;
		os << fblib::utils::folder_append_separator(out_dir)
			<< iter->first.first << "_" << iter->first.second
			<< "_" << iter->second.size() << "_.svg";
		ofstream svg_file(os.str().c_str());
		svg_file << svg_stream.closeSvgFile().str();
		svg_file.close();
	}
	return EXIT_SUCCESS;
}