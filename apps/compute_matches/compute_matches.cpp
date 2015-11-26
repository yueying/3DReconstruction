#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>

#include "fblib/utils/cmd_line.h"
#include "fblib/utils/file_system.h"
#include "fblib/utils/progress.h"

#include "fblib/image/image.h"

#include "fblib/feature/features.h"
#include "fblib/feature/sift.hpp"
#include "fblib/feature/matcher_all_in_memory.h"
#include "fblib/feature/geometric_filter.h"
#include "fblib/feature/pairwise_adjacency_display.h"
#include "fblib/feature/image_list_io_helper.h"
#include "fblib/feature/matcher_brute_force.h"
#include "fblib/feature/matcher_kdtree_flann.h"
#include "fblib/feature/indexed_match_utils.h"

#include "fblib/multiview/fundamental_acransac.h"
#include "fblib/multiview/essential_acransac.h"
#include "fblib/multiview/homography_acransac.h"

using namespace fblib::utils;
using namespace fblib::feature;
using namespace fblib::multiview;
using namespace std;
/**两幅图像计算的关系*/
enum GeometricModel
{
	FUNDAMENTAL_MATRIX = 0,//基础矩阵
	ESSENTIAL_MATRIX = 1,//本质矩阵
	HOMOGRAPHY_MATRIX = 2//单应矩阵
};

//主要用于判断两幅图像是否有可能有相同的内参，因为畸变的数值变化很大
bool testIntrinsicsEquality(
	IntrinsicCameraInfo const &ci1,
	IntrinsicCameraInfo const &ci2)
{
	return ci1.camera_matrix == ci2.camera_matrix;
}

int main(int argc, char **argv)
{
	CmdLine cmd;

	std::string image_dir;
	std::string out_dir = "";
	std::string geometric_model = "e";
	float distance_ratio = .6f;
	bool is_zoom = false;
	float contrast_threshold = 0.04f;

	cmd.add(make_option('i', image_dir, "imadir"));
	cmd.add(make_option('o', out_dir, "outdir"));
	cmd.add(make_option('r', distance_ratio, "distratio"));
	cmd.add(make_option('s', is_zoom, "isZoom"));
	cmd.add(make_option('p', contrast_threshold, "contrastThreshold"));
	cmd.add(make_option('g', geometric_model, "geometricModel"));

	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Usage: " << argv[0] << '\n'
			<< "[-i|--imadir path] \n"
			<< "[-o|--outdir path] \n"
			<< "\n[Optional]\n"
			<< "[-r|--distratio 0.6] \n"
			<< "[-s|--isZoom 0 or 1] \n"
			<< "[-p|--contrastThreshold 0.04 -> 0.01] \n"
			<< "[-g]--geometricModel f, e or h]"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << " You called : " << std::endl
		<< argv[0] << std::endl
		<< "--imadir " << image_dir << std::endl
		<< "--outdir " << out_dir << std::endl
		<< "--distratio " << distance_ratio << std::endl
		<< "--octminus1 " << is_zoom << std::endl
		<< "--peakThreshold " << contrast_threshold << std::endl
		<< "--geometricModel " << geometric_model << std::endl;

	if (out_dir.empty())  {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}

	GeometricModel geometric_model_to_compute = FUNDAMENTAL_MATRIX;
	std::string geometric_matches_filename = "";
	switch (geometric_model[0])
	{
	case 'f': case 'F':
		geometric_model_to_compute = FUNDAMENTAL_MATRIX;
		geometric_matches_filename = "matches.f.txt";
		break;
	case 'e': case 'E':
		geometric_model_to_compute = ESSENTIAL_MATRIX;
		geometric_matches_filename = "matches.e.txt";
		break;
	case 'h': case 'H':
		geometric_model_to_compute = HOMOGRAPHY_MATRIX;
		geometric_matches_filename = "matches.h.txt";
		break;
	default:
		std::cerr << "Unknown geometric model" << std::endl;
		return EXIT_FAILURE;
	}

	// -------------------------------------------
	// 计算过程
	// a. 导入图像
	// b. 计算特征和描述
	// c. 计算可能的匹配
	// d. 通过几何性质过滤可能的匹配
	// e. 导出匹配统计信息
	// -------------------------------------------

	// 创建输出目录
	if (!fblib::utils::folder_exists(out_dir))
		fblib::utils::folder_create(out_dir);

	//根据list.txt文件获取图像集的焦距信息
	std::string file_lists = fblib::utils::create_filespec(out_dir, "lists.txt");
	if (!fblib::utils::is_file(file_lists)) {
		std::cerr << std::endl
			<< "The input file \"" << file_lists << "\" is missing" << std::endl;
		return false;
	}

	std::vector<fblib::feature::CameraInfo> vec_camera_info;
	std::vector<fblib::feature::IntrinsicCameraInfo> vec_cameras_intrinsic;
	if (!fblib::feature::loadImageList(file_lists, vec_camera_info,
		vec_cameras_intrinsic
		))
	{
		std::cerr << "\nEmpty image list." << std::endl;
		return false;
	}

	if (geometric_model_to_compute == ESSENTIAL_MATRIX)
	{
		// 对所有图像的相机矩阵信息进行去重，目前只处理具有相同内参的图片
		std::vector<fblib::feature::IntrinsicCameraInfo>::iterator iterE =
			std::unique(vec_cameras_intrinsic.begin(), vec_cameras_intrinsic.end(), testIntrinsicsEquality);
		vec_cameras_intrinsic.resize(std::distance(vec_cameras_intrinsic.begin(), iterE));
		if (vec_cameras_intrinsic.size() == 1) {
			// 将所有图片的id设为0，即只获取第一幅图像的内参信息即可
			for (size_t i = 0; i < vec_camera_info.size(); ++i)
				vec_camera_info[i].intrinsic_id = 0;
		}
		else  {
			std::cerr << "There is more than one focal group in the lists.txt file." << std::endl
				<< "Only one focal group is supported for the image collection robust essential matrix estimation." << std::endl;
			return EXIT_FAILURE;
		}
	}

	// 两个别名，以便方便的访问图像文件名及图像大小
	std::vector<std::string> file_names;
	std::vector<std::pair<size_t, size_t> > vec_images_size;
	for (std::vector<fblib::feature::CameraInfo>::const_iterator
		iter_camera_info = vec_camera_info.begin();
		iter_camera_info != vec_camera_info.end();
	iter_camera_info++)
	{
		vec_images_size.push_back(std::make_pair(vec_cameras_intrinsic[iter_camera_info->intrinsic_id].width,
			vec_cameras_intrinsic[iter_camera_info->intrinsic_id].height));
		file_names.push_back(fblib::utils::create_filespec(image_dir, iter_camera_info->image_name));
	}

	//计算特征和描述子，如果特征已经计算，则导入特征，否则重新计算并保存
	typedef Descriptor<unsigned char, 128> DescriptorT;
	typedef ScalePointFeature FeatureT;
	typedef std::vector<FeatureT> FeatsT;
	typedef std::vector<DescriptorT > DescsT;
	typedef KeypointSet<FeatsT, DescsT > KeypointSetT;

	{
		std::cout << "\n\nEXTRACT FEATURES" << std::endl;
		vec_images_size.resize(file_names.size());

		Image<unsigned char> gray_image;
		// 显示处理百分比
		ControlProgressDisplay my_progress_bar(file_names.size());
		for (size_t i = 0; i < file_names.size(); ++i)  {
			std::string feat = fblib::utils::create_filespec(out_dir,
				fblib::utils::basename_part(file_names[i]), "feat");
			std::string desc = fblib::utils::create_filespec(out_dir,
				fblib::utils::basename_part(file_names[i]), "desc");

			//如果文件夹下不存在特征及描述，则进行计算
			if (!fblib::utils::file_exists(feat) || !fblib::utils::file_exists(desc)) {

				if (!readImage(file_names[i].c_str(), &gray_image))
					continue;

				// 计算特征和描述，然后将他们导入到文件中
				KeypointSetT keypoint_set;
				SIFTDetector(gray_image,
					keypoint_set.features(), keypoint_set.descriptors(),
					is_zoom, true, contrast_threshold);
				keypoint_set.saveToBinFile(feat, desc);
			}
			++my_progress_bar;
		}
	}

	// 计算可能的匹配
	PairWiseMatches map_putatives_matches;

	// 定义匹配算子采用(Squared L2)采用ANN
	typedef flann::L2<DescriptorT::bin_type> MetricT;
	typedef ArrayMatcherKdtreeFlann<DescriptorT::bin_type, MetricT> MatcherT;

	// 或者可以采用暴力匹配，具体如下：
	//typedef SquaredEuclideanDistanceVectorized<DescriptorT::bin_type> MetricT;
	//typedef ArrayMatcherBruteForce<DescriptorT::bin_type, MetricT> MatcherT;

	// 如果匹配已经存在，重新导入
	if (fblib::utils::file_exists(out_dir + "/matches.putative.txt"))
	{
		pairedIndexedMatchImport(out_dir + "/matches.putative.txt", map_putatives_matches);
		std::cout << std::endl << "PUTATIVE MATCHES -- PREVIOUS RESULTS LOADED" << std::endl;
	}
	else // 计算匹配
	{
		MatcherAllInMemory<KeypointSetT, MatcherT> collectionMatcher(distance_ratio);
		if (collectionMatcher.LoadData(file_names, out_dir))
		{
			std::cout << std::endl << "PUTATIVE MATCHES" << std::endl;
			collectionMatcher.Match(file_names, map_putatives_matches);
			// 导出可能的匹配
			std::ofstream file(std::string(out_dir + "/matches.putative.txt").c_str());
			if (file.is_open())
				PairedIndexedMatchToStream(map_putatives_matches, file);
			file.close();
		}
	}
	//导出可能的匹配，通过邻接矩阵的方式显示
	PairWiseMatchingToAdjacencyMatrixSVG(file_names.size(),
		map_putatives_matches,
		fblib::utils::create_filespec(out_dir, "PutativeAdjacencyMatrix", "svg"));

	// 根据几何性质对可能的匹配进行过滤
	PairWiseMatches map_geometric_matches;

	ImageCollectionGeometricFilter<FeatureT> collection_geom_filter;
	const double max_residual_error = 4.0;
	if (collection_geom_filter.LoadData(file_names, out_dir))
	{
		std::cout << std::endl << " - GEOMETRIC FILTERING - " << std::endl;
		switch (geometric_model_to_compute)
		{
		case FUNDAMENTAL_MATRIX:
		{
			collection_geom_filter.Filter(
				GeometricFilter_FMatrix_AC(max_residual_error),
				map_putatives_matches,
				map_geometric_matches,
				vec_images_size);
		}
			break;
		case ESSENTIAL_MATRIX:
		{
			collection_geom_filter.Filter(
				GeometricFilter_EMatrix_AC(vec_cameras_intrinsic[0].camera_matrix, max_residual_error),
				map_putatives_matches,
				map_geometric_matches,
				vec_images_size);

			//进行额外的检查，用于移除比较差的重叠
			std::vector<PairWiseMatches::key_type> vec_to_remove;
			for (PairWiseMatches::const_iterator iter_map = map_geometric_matches.begin();
				iter_map != map_geometric_matches.end(); ++iter_map)
			{
				size_t putative_photometric_count = map_putatives_matches.find(iter_map->first)->second.size();
				size_t putative_geometric_count = iter_map->second.size();
				float ratio = putative_geometric_count / (float)putative_photometric_count;
				if (putative_geometric_count < 50 || ratio < .3f)  {
					//添加到移除向量中
					vec_to_remove.push_back(iter_map->first);
				}
			}
			//移除废弃的匹配对
			for (std::vector<PairWiseMatches::key_type>::const_iterator
				iter = vec_to_remove.begin(); iter != vec_to_remove.end(); ++iter)
			{
				map_geometric_matches.erase(*iter);
			}
		}
			break;
		case HOMOGRAPHY_MATRIX:
		{

			collection_geom_filter.Filter(
				GeometricFilter_HMatrix_AC(max_residual_error),
				map_putatives_matches,
				map_geometric_matches,
				vec_images_size);
		}
			break;
		}

		// 导出根据几何性质过滤之后的匹配
		std::ofstream file(string(out_dir + "/" + geometric_matches_filename).c_str());
		if (file.is_open())
			PairedIndexedMatchToStream(map_geometric_matches, file);
		file.close();

		// 导出邻接矩阵
		std::cout << "\n Export Adjacency Matrix of the pairwise's geometric matches"
			<< std::endl;
		PairWiseMatchingToAdjacencyMatrixSVG(file_names.size(),
			map_geometric_matches,
			fblib::utils::create_filespec(out_dir, "GeometricAdjacencyMatrix", "svg"));
	}
	return EXIT_SUCCESS;
}