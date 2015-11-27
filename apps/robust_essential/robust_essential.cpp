#include "mvg/camera/pinhole_camera.h"
#include "mvg/image/image.h"
#include "mvg/feature/features.h"
#include "mvg/feature/matcher_brute_force.h"
#include "mvg/feature/indexed_match_decorator.h"
#include "mvg/camera/projection.h"
#include "mvg/multiview/triangulation.h"
#include "mvg/multiview/essential_estimation.h"
#include "mvg/feature/sift.hpp"
#include "mvg/feature/two_view_matches.h"

#include "mvg/utils/file_system.h"
#include "mvg/utils/svg_drawer.h"

#include <string>
#include <iostream>

using namespace mvg::utils;
using namespace mvg::feature;
using namespace mvg::camera;
using namespace mvg::math;
using namespace mvg::multiview;


/**	从(ASCII)文件中读取相机内参
 *	fx 0 cx
 *  0 fy cy
 *  0 0 1
 */
bool readIntrinsic(const std::string &file_name, Mat3 &camera_matrix);

/** 将3D点和相机位置数据导入到PLY文件中
 */
bool exportToPly(const std::vector<Vec3> &vec_points,
	const std::vector<Vec3> &vec_camera_pose,
	const std::string &file_name);

/**	通过三角化导出数据点位ply格式（点在相机前）
 */
void triangulateAndSaveResult(
	const PinholeCamera &left_camera,
	const PinholeCamera &right_camera,
	const std::vector<size_t> &vec_inliers,
	const Mat &left_points,
	const Mat &right_points,
	std::vector<Vec3> &vec_3d_points);

int main(int argc, char *argv[]) {

	std::string input_dir = string(THIS_SOURCE_DIR)
		+ "/data/imageData/SceauxCastle/";
	std::string left_image_name = input_dir + "100_7101.jpg";
	std::string right_image_name = input_dir + "100_7102.jpg";

	Image<unsigned char> left_image, right_image;
	readImage(left_image_name.c_str(), &left_image);
	readImage(right_image_name.c_str(), &right_image);

	// 定义使用的描述子(SIFT : 128 float类型值)
	typedef float descType;
	typedef Descriptor<descType, 128> SIFTDescriptor;

	// 定义vector用来存储检测到的特征和对应描述子
	std::vector<ScalePointFeature> left_features, right_features;
	std::vector<SIFTDescriptor > left_descriptors, right_descriptors;
	// 计算SIFT特征，设置计算特征参数
	bool is_zoom = false;
	bool is_root_sift = true;
	SIFTDetector(left_image, left_features, left_descriptors, is_zoom, is_root_sift);
	SIFTDetector(right_image, right_features, right_descriptors, is_zoom, is_root_sift);

	// 将左右图像合并显示进行对比
	{
		Image<unsigned char> concat;
		concatHorizontal(left_image, right_image, concat);
		std::string out_filename = "01_concat.jpg";
		writeImage(out_filename.c_str(), concat);
	}

	// 画出左右两幅图像的特征
  {
	  Image<unsigned char> concat;
	  concatHorizontal(left_image, right_image, concat);

	  // 画出特征 :
	  for (size_t i = 0; i < left_features.size(); ++i)  {
		  const ScalePointFeature &left_img = left_features[i];
		  DrawCircle(left_img.x(), left_img.y(), left_img.scale(), 255, &concat);
	  }
	  for (size_t i = 0; i < right_features.size(); ++i)  {
		  const ScalePointFeature & right_img = right_features[i];
		  DrawCircle(right_img.x() + left_image.Width(), right_img.y(), right_img.scale(), 255, &concat);
	  }
	  std::string out_filename = "02_features.jpg";
	  writeImage(out_filename.c_str(), concat);
  }

	std::vector<IndexedMatch> vec_putative_matches;
	// 执行特征匹配，找最近邻匹配，通过距离比进行过滤
	{
		//定义匹配的度量标准，采用欧式距离的平方
		typedef SquaredEuclideanDistanceVectorized<SIFTDescriptor::bin_type> Metric;
		// 定义暴力匹配
		typedef ArrayMatcherBruteForce<SIFTDescriptor::bin_type, Metric> MatcherT;
		// 设定距离比率进行匹配
		GetPutativesMatches<SIFTDescriptor, MatcherT>(left_descriptors, right_descriptors, Square(0.8), vec_putative_matches);

		IndexedMatchDecorator<float> match_deduplicator(
			vec_putative_matches, left_features, right_features);
		match_deduplicator.getDeduplicated(vec_putative_matches);

		// 距离比率过后画出相对于的匹配
		SvgDrawer svg_stream(left_image.Width() + right_image.Width(), max(left_image.Height(), right_image.Height()));
		svg_stream.drawImage(left_image_name, left_image.Width(), left_image.Height());
		svg_stream.drawImage(right_image_name, right_image.Width(), right_image.Height(), left_image.Width());
		for (size_t i = 0; i < vec_putative_matches.size(); ++i) {
			//得到特征，绘制圆用直线进行连接
			const ScalePointFeature & left_feature = left_features[vec_putative_matches[i]._i];
			const ScalePointFeature & right_feature = right_features[vec_putative_matches[i]._j];
			svg_stream.drawLine(left_feature.x(), left_feature.y(), right_feature.x() + left_image.Width(), right_feature.y(), SvgStyle().stroke("green", 2.0));
			svg_stream.drawCircle(left_feature.x(), left_feature.y(), left_feature.scale(), SvgStyle().stroke("yellow", 2.0));
			svg_stream.drawCircle(right_feature.x() + left_image.Width(), right_feature.y(), right_feature.scale(), SvgStyle().stroke("yellow", 2.0));
		}
		std::string out_filename = "03_siftMatches.svg";
		std::ofstream svg_file(out_filename.c_str());
		svg_file << svg_stream.closeSvgFile().str();
		svg_file.close();
	}

	// 通过两图像之间的本质矩阵对匹配对进行过滤
  {
	  Mat3 camera_matrix;
	  //读取相机内参矩阵
	  if (!readIntrinsic(mvg::utils::create_filespec(input_dir, "camera_matrix", "txt"), camera_matrix))
	  {
		  std::cerr << "Cannot read intrinsic parameters." << std::endl;
		  return EXIT_FAILURE;
	  }

	  //A. 准备匹配的对应点
	  Mat left_points(2, vec_putative_matches.size());
	  Mat right_points(2, vec_putative_matches.size());
	  for (size_t k = 0; k < vec_putative_matches.size(); ++k)  {
		  const ScalePointFeature & left_feature = left_features[vec_putative_matches[k]._i];
		  const ScalePointFeature & right_feature = right_features[vec_putative_matches[k]._j];
		  left_points.col(k) = left_feature.coords().cast<double>();
		  right_points.col(k) = right_feature.coords().cast<double>();
	  }

	  //B. 本质矩阵的鲁棒性估计 
	  std::vector<size_t> vec_inliers;
	  Mat3 essential_matrix;
	  std::pair<size_t, size_t> left_image_size(left_image.Width(), left_image.Height());
	  std::pair<size_t, size_t> right_image_size(right_image.Width(), right_image.Height());
	  double thresholdE = 0.0, NFA = 0.0;
	  if (robustEssential(
		  camera_matrix, camera_matrix,         // 相机矩阵
		  left_points, right_points,       // 左右对应点
		  &essential_matrix,           // 本质矩阵
		  &vec_inliers, // inliers 
		  left_image_size,    // 左图像大小
		  right_image_size,    // 右图像大小
		  &thresholdE,  // Found AContrario Theshold
		  &NFA,         // Found AContrario NFA
		  std::numeric_limits<double>::infinity()))
	  {
		  std::cout << "\nFound an Essential matrix under the confidence threshold of: "
			  << thresholdE << " pixels\n\twith: " << vec_inliers.size() << " inliers"
			  << " from: " << vec_putative_matches.size()
			  << " putatives correspondences"
			  << std::endl;

		  // 显示通过本质矩阵约束之后的匹配点对
		  SvgDrawer svg_stream(left_image.Width() + right_image.Width(), max(left_image.Height(), right_image.Height()));
		  svg_stream.drawImage(left_image_name, left_image.Width(), left_image.Height());
		  svg_stream.drawImage(right_image_name, right_image.Width(), right_image.Height(), left_image.Width());
		  for (size_t i = 0; i < vec_inliers.size(); ++i)  {
			  const ScalePointFeature & left_feature = left_features[vec_putative_matches[vec_inliers[i]]._i];
			  const ScalePointFeature & right_feature = right_features[vec_putative_matches[vec_inliers[i]]._j];
			  const Vec2f left_feature_coords = left_feature.coords();
			  const Vec2f right_feature_coords = right_feature.coords();
			  svg_stream.drawLine(left_feature_coords.x(), left_feature_coords.y(), right_feature_coords.x() + left_image.Width(), right_feature_coords.y(), SvgStyle().stroke("green", 2.0));
			  svg_stream.drawCircle(left_feature_coords.x(), left_feature_coords.y(), left_feature.scale(), SvgStyle().stroke("yellow", 2.0));
			  svg_stream.drawCircle(right_feature_coords.x() + left_image.Width(), right_feature_coords.y(), right_feature.scale(), SvgStyle().stroke("yellow", 2.0));
		  }
		  std::string out_filename = "04_ACRansacEssential.svg";
		  std::ofstream svg_file(out_filename.c_str());
		  svg_file << svg_stream.closeSvgFile().str();
		  svg_file.close();

		  //根据本质矩阵计算出相机的外参
		  Mat3 R;
		  Vec3 t;
		  if (!estimateRtFromE(camera_matrix, camera_matrix, left_points, right_points, essential_matrix, vec_inliers,
			  &R, &t))
		  {
			  std::cerr << " /!\\ Failed to compute initial right_feature|t for the initial pair" << std::endl;
			  return false;
		  }
		  std::cout << std::endl
			  << "-- Rotation|Translation matrices: --" << std::endl
			  << R << std::endl << std::endl << t << std::endl;

		  // 构建左右相机
		  PinholeCamera left_camera(camera_matrix, Mat3::Identity(), Vec3::Zero());
		  PinholeCamera right_camera(camera_matrix, R, t);

		  // 通过三角定位计算三维点
		  std::vector<Vec3> vec_3d_points;
		  triangulateAndSaveResult(
			  left_camera, right_camera,
			  vec_inliers,
			  left_points, right_points, vec_3d_points);

		  // 将相机位置和3d点导出，成ply文件
		  std::vector<Vec3> vec_camera_pose;
		  vec_camera_pose.push_back(left_camera.camera_center_);
		  vec_camera_pose.push_back(right_camera.camera_center_);
		  exportToPly(vec_3d_points, vec_camera_pose, "EssentialGeometry.ply");

	  }
	  else  {
		  std::cout << "ACRANSAC was unable to estimate a rigid essential matrix"
			  << std::endl;
	  }
  }
	return EXIT_SUCCESS;
}

// 读取相机内参
bool readIntrinsic(const std::string & file_name, Mat3 & camera_matrix)
{
	std::ifstream in;
	in.open(file_name.c_str(), std::ifstream::in);
	if (in.is_open())  {
		for (int j = 0; j < 3; ++j)
			for (int i = 0; i < 3; ++i)
				in >> camera_matrix(j, i);
	}
	else  {
		std::cerr << std::endl
			<< "Invalid input camera_matrix.txt file" << std::endl;
		return false;
	}
	return true;
}

// 将形成的3D点和相机位置导入到PLY格式中
bool exportToPly(const std::vector<Vec3> & vec_points,
	const std::vector<Vec3> & vec_camera_pose,
	const std::string & file_name)
{
	std::ofstream outfile;
	outfile.open(file_name.c_str(), std::ios_base::out);

	outfile << "ply"
		<< '\n' << "format ascii 1.0"
		<< '\n' << "element vertex " << vec_points.size() + vec_camera_pose.size()
		<< '\n' << "property float x"
		<< '\n' << "property float y"
		<< '\n' << "property float z"
		<< '\n' << "property uchar red"
		<< '\n' << "property uchar green"
		<< '\n' << "property uchar blue"
		<< '\n' << "end_header" << std::endl;

	for (size_t i = 0; i < vec_points.size(); ++i)  {
		outfile << vec_points[i].transpose()
			<< " 255 255 255" << "\n";
	}

	for (size_t i = 0; i < vec_camera_pose.size(); ++i)  {
		outfile << vec_camera_pose[i].transpose()
			<< " 0 255 0" << "\n";
	}
	outfile.flush();
	bool is_ok = outfile.good();
	outfile.close();
	return is_ok;
}

/**	通过三角定位，确定有效3d点，导出到PLY中
 */
void triangulateAndSaveResult(
	const PinholeCamera &left_camera,
	const PinholeCamera &right_camera,
	const std::vector<size_t> &vec_inliers,
	const Mat & left_points,
	const Mat & right_points,
	std::vector<Vec3> &vec_3d_points)
{
	std::vector<double> vec_residuals;
	size_t PointWithNegativeDepth = 0;
	for (size_t k = 0; k < vec_inliers.size(); ++k) {
		const Vec2 & left_point = left_points.col(vec_inliers[k]);
		const Vec2 & right_point = right_points.col(vec_inliers[k]);

		Vec3 world_point = Vec3::Zero();
		TriangulateDLT(left_camera.projection_matrix_, left_point, right_camera.projection_matrix_, right_point, &world_point);

		// 计算投影误差
		double projection_residual = (left_camera.Residual(world_point, left_point) + right_camera.Residual(world_point, right_point)) / 2.0;
		vec_residuals.push_back(projection_residual);
		if (left_camera.Depth(world_point) < 0 && right_camera.Depth(world_point) < 0) {
			++PointWithNegativeDepth;
		}
		else  {
			vec_3d_points.push_back(world_point);
		}
	}
	if (PointWithNegativeDepth > 0)
	{
		std::cout << PointWithNegativeDepth
			<< " correspondence(s) with negative depth have been discarded."
			<< std::endl;
	}
	//对投影误差进行统计分析 显示
	float min_residual , max_residual , meam_residual , median_residual ;
	MinMaxMeanMedian<float>(vec_residuals.begin(), vec_residuals.end(),
		min_residual , max_residual , meam_residual , median_residual );

	std::cout << std::endl
		<< "Essential matrix estimation, residuals statistics:" << "\n"
		<< "\t-- Residual min:\t" << min_residual  << std::endl
		<< "\t-- Residual median:\t" << median_residual  << std::endl
		<< "\t-- Residual max:\t " << max_residual  << std::endl
		<< "\t-- Residual mean:\t " << meam_residual  << std::endl;
}
