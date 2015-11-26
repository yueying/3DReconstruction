#include "fblib/camera/pinhole_camera.h"
#include "fblib/image/image.h"
#include "fblib/feature/features.h"
#include "fblib/feature/matcher_brute_force.h"
#include "fblib/feature/indexed_match_decorator.h"
#include "fblib/camera/projection.h"
#include "fblib/multiview/triangulation.h"
#include "fblib/multiview/essential_estimation.h"

#include "fblib/feature/sift.hpp"
#include "fblib/feature/two_view_matches.h"

#include "fblib/utils/file_system.h"
#include "fblib/utils/svg_drawer.h"

#include "fblib/sfm/problem_data_container.h"
#include "fblib/multiview/solver_resection_p3p.h"
#include "fblib/sfm/pinhole_brown_rt_ceres_functor.h"
#include "fblib/sfm/pinhole_ceres_functor.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <string>
#include <iostream>

using namespace fblib::utils;
using namespace fblib::feature;
using namespace fblib::camera;
using namespace fblib::multiview;
using namespace fblib::sfm;

/// Read intrinsic camera_matrix matrix from a file (ASCII)
/// fundamental_matrix 0 ppx
/// 0 fundamental_matrix ppy
/// 0 0 1
bool readIntrinsic(const std::string & fileName, Mat3 & camera_matrix);

/// Export 3D point vector and camera position to PLY format
bool exportToPly(const std::vector<Vec3> & vec_points,
  const std::vector<Vec3> & vec_camera_pose,
  const std::string & file_name);

/// Triangulate and export valid point as PLY (point in front of the cameras)
void triangulateAndSaveResult(
  const PinholeCamera & left_camera,
  const PinholeCamera & right_camera,
  std::vector<size_t> & vec_inliers,
  const Mat & left_points,
  const Mat & right_points,
  std::vector<Vec3> & vec_3d_points);

/// Perform a Bundle Adjustment: Refine the camera [R|t|focal] and the structure
void do_bundle_adjustment(
  PinholeCamera & left_camera,
  PinholeCamera & right_camera,
  const Mat & left_points,
  const Mat & right_points,
  const std::vector<size_t> & vec_inliers,
  std::vector<Vec3> & vec_3d_points);

/// Perform a Bundle Adjustment: Refine the cameras [R|t]
///  a common [focal] and the structure
void do_bundle_adjustment_common_focal(
  PinholeCamera & left_camera,
  PinholeCamera & right_camera,
  const Mat & left_points,
  const Mat & right_points,
  const std::vector<size_t> & vec_inliers,
  std::vector<Vec3> & vec_3d_points);

/// Perform a Bundle Adjustment: Refine the cameras [R|t]
///  and common intrinsics [focal,ppx,ppy,k1,k2,k3] and the structure
void do_bundle_adjustment_common_intrinsics(
  PinholeCamera & left_camera,
  PinholeCamera & right_camera,
  const Mat & left_points,
  const Mat & right_points,
  const std::vector<size_t> & vec_inliers,
  std::vector<Vec3> & vec_3d_points);

/// Show :
///  how computing an essential with know internal calibration matrix camera_matrix
///  how refine the camera motion, focal and structure with Bundle Adjustment
///   way 1: independent cameras [R|t|f] and structure
///   way 2: independent cameras motion [R|t], shared focal [f] and structure
int main() {

  std::string input_dir = string(THIS_SOURCE_DIR)
    + "/data/imageData/SceauxCastle/";
  Image<RGBColor> image;
  string left_image_name = input_dir + "100_7101.jpg";
  string right_image_name = input_dir + "100_7102.jpg";

  Image<unsigned char> left_image, right_image;
  readImage(left_image_name.c_str(), &left_image);
  readImage(right_image_name.c_str(), &right_image);

  // 定义使用的描述子(SIFT : 128 float类型值)
  typedef float descType;
  typedef Descriptor<descType,128> SIFTDescriptor;

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
    string out_filename = "01_concat.jpg";
    writeImage(out_filename.c_str(), concat);
  }

  // 画出左右两幅图像的特征
  {
    Image<unsigned char> concat;
    concatHorizontal(left_image, right_image, concat);

	// 画出特征 :
    for (size_t i=0; i < left_features.size(); ++i )  {
      const ScalePointFeature & left_img = left_features[i];
      DrawCircle(left_img.x(), left_img.y(), left_img.scale(), 255, &concat);
    }
    for (size_t i=0; i < right_features.size(); ++i )  {
      const ScalePointFeature & right_img = right_features[i];
      DrawCircle(right_img.x()+left_image.Width(), right_img.y(), right_img.scale(), 255, &concat);
    }
    string out_filename = "02_features.jpg";
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

    IndexedMatchDecorator<float> matchDeduplicator(
            vec_putative_matches, left_features, right_features);
    matchDeduplicator.getDeduplicated(vec_putative_matches);

	// 距离比率过后画出相对于的匹配
    SvgDrawer svg_stream( left_image.Width() + right_image.Width(), max(left_image.Height(), right_image.Height()));
    svg_stream.drawImage(left_image_name, left_image.Width(), left_image.Height());
    svg_stream.drawImage(right_image_name, right_image.Width(), right_image.Height(), left_image.Width());
    for (size_t i = 0; i < vec_putative_matches.size(); ++i) {
		//得到特征，绘制圆用直线进行连接
      const ScalePointFeature & L = left_features[vec_putative_matches[i]._i];
      const ScalePointFeature & R = right_features[vec_putative_matches[i]._j];
      svg_stream.drawLine(L.x(), L.y(), R.x()+left_image.Width(), R.y(), SvgStyle().stroke("green", 2.0));
      svg_stream.drawCircle(L.x(), L.y(), L.scale(), SvgStyle().stroke("yellow", 2.0));
      svg_stream.drawCircle(R.x()+left_image.Width(), R.y(), R.scale(),SvgStyle().stroke("yellow", 2.0));
    }
    string out_filename = "03_siftMatches.svg";
    ofstream svg_file( out_filename.c_str() );
    svg_file << svg_stream.closeSvgFile().str();
    svg_file.close();
  }

  // 通过两图像之间的本质矩阵对匹配对进行过滤
  {
    Mat3 camera_matrix;
	//读取相机内参矩阵
    if (!readIntrinsic(fblib::utils::create_filespec(input_dir,"camera_matrix","txt"), camera_matrix))
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
    std::pair<size_t, size_t> size_imaL(left_image.Width(), left_image.Height());
    std::pair<size_t, size_t> size_imaR(right_image.Width(), right_image.Height());
    double thresholdE = 0.0, NFA = 0.0;
    if (robustEssential(
      camera_matrix, camera_matrix,         // intrinsics
      left_points, right_points,       // corresponding points
      &essential_matrix,           // essential matrix
      &vec_inliers, // inliers
      size_imaL,    // Left image size
      size_imaR,    // Right image size
      &thresholdE,  // Found AContrario Theshold
      &NFA,         // Found AContrario NFA
      std::numeric_limits<double>::infinity()))
    {
      std::cout << "\nFound an Essential matrix under the confidence threshold of: "
        << thresholdE << " pixels\n\twith: " << vec_inliers.size() << " inliers"
        << " from: " << vec_putative_matches.size()
        << " putatives correspondences"
        << std::endl;

      // Show Essential validated point
      SvgDrawer svg_stream( left_image.Width() + right_image.Width(), max(left_image.Height(), right_image.Height()));
      svg_stream.drawImage(left_image_name, left_image.Width(), left_image.Height());
      svg_stream.drawImage(right_image_name, right_image.Width(), right_image.Height(), left_image.Width());
      for ( size_t i = 0; i < vec_inliers.size(); ++i)  {
        const ScalePointFeature & LL = left_features[vec_putative_matches[vec_inliers[i]]._i];
        const ScalePointFeature & RR = right_features[vec_putative_matches[vec_inliers[i]]._j];
        const Vec2f L = LL.coords();
        const Vec2f R = RR.coords();
        svg_stream.drawLine(L.x(), L.y(), R.x()+left_image.Width(), R.y(), SvgStyle().stroke("green", 2.0));
        svg_stream.drawCircle(L.x(), L.y(), LL.scale(), SvgStyle().stroke("yellow", 2.0));
        svg_stream.drawCircle(R.x()+left_image.Width(), R.y(), RR.scale(),SvgStyle().stroke("yellow", 2.0));
      }
      std::string out_filename = "04_ACRansacEssential.svg";
      std::ofstream svg_file( out_filename.c_str() );
      svg_file << svg_stream.closeSvgFile().str();
      svg_file.close();

      //C. Extract the rotation and translation of the camera from the essential matrix
      Mat3 R;
      Vec3 t;
      if (!estimateRtFromE(camera_matrix, camera_matrix, left_points, right_points, essential_matrix, vec_inliers,
        &R, &t))
      {
        std::cerr << " /!\\ Failed to compute initial R|t for the initial pair" << std::endl;
        return false;
      }
      std::cout << std::endl
        << "-- Rotation|Translation matrices: --" << std::endl
        << R << std::endl << std::endl << t << std::endl;

      // Build Left and Right Camera
      PinholeCamera left_camera(camera_matrix, Mat3::Identity(), Vec3::Zero());
      PinholeCamera right_camera(camera_matrix, R, t);

      //C. Triangulate and check valid points
      // invalid point that do not respect cheirality are discarded (removed
      //  from the list of inliers.
      std::vector<Vec3> vec_3d_points;
      triangulateAndSaveResult(
        left_camera, right_camera,
        vec_inliers,
        left_points, right_points, vec_3d_points);

      //D. Refine the computed structure and cameras
      std::cout << "Which BA do you want ? " << std::endl
        << "\t 1: Refine [X],[f,R|t] (individual cameras)\n"
        << "\t 2: Refine [X],[R|t], shared [f]\n"
        << "\t 3: Refine [X],[R|t], shared brown disto models [f,ppx,ppy,k1,k2,k3]\n" << std::endl;
      int iBAType = -1;
      std::cin >> iBAType;
      switch(iBAType)
      {
        case 1:
        {
          do_bundle_adjustment(
            left_camera, right_camera,
            left_points, right_points,
            vec_inliers,
            vec_3d_points);
        }
        break;

        case 2:
        {
          do_bundle_adjustment_common_focal(
            left_camera, right_camera,
            left_points, right_points,
            vec_inliers,
            vec_3d_points);
        }
        break;

        case 3:
        {
          do_bundle_adjustment_common_intrinsics(
            left_camera, right_camera,
            left_points, right_points,
            vec_inliers,
            vec_3d_points);
        }
        break;
        default:
          std::cerr << "Invalid input number" << std::endl;
      }


      //essential_matrix. Export as PLY the refined scene (camera pos + 3Dpoints)
      std::vector<Vec3> vec_camera_pose;
      vec_camera_pose.push_back( left_camera.camera_center_ );
      vec_camera_pose.push_back( right_camera.camera_center_ );
      exportToPly(vec_3d_points, vec_camera_pose, "EssentialGeometry.ply");

    }
    else  {
      std::cout << "ACRANSAC was unable to estimate a rigid essential matrix"
        << std::endl;
    }
  }
  return EXIT_SUCCESS;
}

bool readIntrinsic(const std::string & fileName, Mat3 & camera_matrix)
{
  // Load the camera_matrix matrix
  ifstream in;
  in.open( fileName.c_str(), ifstream::in);
  if(in.is_open())  {
    for (int j=0; j < 3; ++j)
      for (int i=0; i < 3; ++i)
        in >> camera_matrix(j,i);
  }
  else  {
    std::cerr << std::endl
      << "Invalid input camera_matrix.txt file" << std::endl;
    return false;
  }
  return true;
}

/// Export 3D point vector and camera position to PLY format
bool exportToPly(const std::vector<Vec3> & vec_points,
  const std::vector<Vec3> & vec_camera_pose,
  const std::string & file_name)
{
  std::ofstream outfile;
  outfile.open(file_name.c_str(), std::ios_base::out);

  outfile << "ply"
    << '\n' << "format ascii 1.0"
    << '\n' << "element vertex " << vec_points.size()+vec_camera_pose.size()
    << '\n' << "property float x"
    << '\n' << "property float y"
    << '\n' << "property float z"
    << '\n' << "property uchar red"
    << '\n' << "property uchar green"
    << '\n' << "property uchar blue"
    << '\n' << "end_header" << std::endl;

  for (size_t i=0; i < vec_points.size(); ++i)  {
      outfile << vec_points[i].transpose()
      << " 255 255 255" << "\n";
  }

  for (size_t i=0; i < vec_camera_pose.size(); ++i)  {
    outfile << vec_camera_pose[i].transpose()
      << " 0 255 0" << "\n";
  }
  outfile.flush();
  bool is_ok = outfile.good();
  outfile.close();
  return is_ok;
}

/// Triangulate and export valid point as PLY (point in front of the cameras)
void triangulateAndSaveResult(
  const PinholeCamera & left_camera,
  const PinholeCamera & right_camera,
  std::vector<size_t> & vec_inliers,
  const Mat & left_points,
  const Mat & right_points,
  std::vector<Vec3> & vec_3d_points)
{
  size_t nb_invalid3DPoints = 0;
  std::vector<size_t> vec_valid3DPoints;
  std::vector<double> vec_residuals;
  for (size_t k = 0; k < vec_inliers.size(); ++k) {
    const Vec2 & xL_ = left_points.col(vec_inliers[k]);
    const Vec2 & xR_ = right_points.col(vec_inliers[k]);

    Vec3 X = Vec3::Zero();
    TriangulateDLT(left_camera.projection_matrix_, xL_, right_camera.projection_matrix_, xR_, &X);

    // Compute residual:
    double dResidual = (left_camera.Residual(X, xL_) + right_camera.Residual(X, xR_))/2.0;
    vec_residuals.push_back(dResidual);
    if (left_camera.Depth(X) < 0 && right_camera.Depth(X) < 0) {
      ++nb_invalid3DPoints;
    }
    else  {
      vec_3d_points.push_back(X);
      vec_valid3DPoints.push_back(vec_inliers[k]);
    }
  }
  if (nb_invalid3DPoints > 0)
  {
    std::cout << nb_invalid3DPoints
      << " correspondence(s) with negative depth have been discarded."
      << std::endl;
    // remove from the inliers list the point that are behind the camera
    vec_inliers = vec_valid3DPoints;
  }

  // Display some statistics of reprojection errors
  float min_residual, max_residual, mean_residual, median_residual;
  MinMaxMeanMedian<float>(vec_residuals.begin(), vec_residuals.end(),
                        min_residual, max_residual, mean_residual, median_residual);

  std::cout << std::endl
    << "Essential matrix estimation, residuals statistics:" << "\n"
    << "\t-- Residual min:\t" << min_residual << std::endl
    << "\t-- Residual median:\t" << median_residual << std::endl
    << "\t-- Residual max:\t "  << max_residual << std::endl
    << "\t-- Residual mean:\t " << mean_residual << std::endl;
}

void do_bundle_adjustment(
  PinholeCamera & left_camera,
  PinholeCamera & right_camera,
  const Mat & left_points,
  const Mat & right_points,
  const std::vector<size_t> & vec_inliers,
  std::vector<Vec3> & vec_3d_points)
{
  int nviews = 2;
  int n3Dpoints = vec_inliers.size();

  // Setup a BA problem
  BAProblemData<7> ba_problem;

  // Configure the size of the problem
  ba_problem.num_cameras_ = nviews;
  ba_problem.num_points_ = n3Dpoints;
  ba_problem.num_observations_ = nviews * n3Dpoints;

  ba_problem.point_index_.reserve(ba_problem.num_observations_);
  ba_problem.camera_index_.reserve(ba_problem.num_observations_);
  ba_problem.observations_.reserve(2 * ba_problem.num_observations_);

  ba_problem.num_parameters_ = 7 * ba_problem.num_cameras_ + 3 * ba_problem.num_points_;
  ba_problem.parameters_.reserve(ba_problem.num_parameters_);

  // fill it with data (For each 3D point setup the tracks : the 2D visbility)
  PinholeCamera vec_cam[2] = {left_camera, right_camera};
  for (int i = 0; i < n3Dpoints; ++i) {
    // Collect the image of point i in each frame (left_points, right_points).
    const Vec2 & xL_ = left_points.col(vec_inliers[i]);
    const Vec2 & xR_ = right_points.col(vec_inliers[i]);

    // Left 2D observations
    double ppx = vec_cam[0].camera_matrix_(0,2), ppy = vec_cam[0].camera_matrix_(1,2);
    ba_problem.camera_index_.push_back(0);
    ba_problem.point_index_.push_back(i);
    ba_problem.observations_.push_back( xL_(0) - ppx);
    ba_problem.observations_.push_back( xL_(1) - ppy);

    // Right 2D observations
    ppx = vec_cam[1].camera_matrix_(0,2);
    ppy = vec_cam[1].camera_matrix_(1,2);
    ba_problem.camera_index_.push_back(1);
    ba_problem.point_index_.push_back(i);
    ba_problem.observations_.push_back( xR_(0) - ppx);
    ba_problem.observations_.push_back( xR_(1) - ppy);
  }

  // Add camera parameters (R, t, focal)
  for (int j = 0; j < nviews; ++j) {
    // Rotation matrix to angle axis
    std::vector<double> angleAxis(3);
    ceres::RotationMatrixToAngleAxis((const double*)vec_cam[j].rotation_matrix_.data(), &angleAxis[0]);
    // translation
    Vec3 t = vec_cam[j].translation_vector_;
    double focal = vec_cam[j].camera_matrix_(0,0);
    ba_problem.parameters_.push_back(angleAxis[0]);
    ba_problem.parameters_.push_back(angleAxis[1]);
    ba_problem.parameters_.push_back(angleAxis[2]);
    ba_problem.parameters_.push_back(t[0]);
    ba_problem.parameters_.push_back(t[1]);
    ba_problem.parameters_.push_back(t[2]);
    ba_problem.parameters_.push_back(focal);
  }

  // Add 3D points coordinates parameters
  for (int i = 0; i < n3Dpoints; ++i) {
    Vec3 point_3d = vec_3d_points[i];
    ba_problem.parameters_.push_back(point_3d[0]);
    ba_problem.parameters_.push_back(point_3d[1]);
    ba_problem.parameters_.push_back(point_3d[2]);
  }

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < ba_problem.num_observations(); ++i) {

    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ErrorFuncRefineCamera3DPointsPinhole, 2, 7, 3>(
		new ErrorFuncRefineCamera3DPointsPinhole(
                & ba_problem.observations()[2 * i]));

    problem.AddResidualBlock(cost_function,
                             NULL, // squared loss
                             ba_problem.mutable_camera_for_observation(i),
                             ba_problem.mutable_point_for_observation(i));
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  else
    if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
      options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
    else
    {
      // No sparse backend for Ceres.
      // Use dense solving
      options.linear_solver_type = ceres::DENSE_SCHUR;
    }
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  double residual_before = std::sqrt( summary.initial_cost / (ba_problem.num_observations_*2.));
  double residual_after = std::sqrt( summary.final_cost / (ba_problem.num_observations_*2.));

  std::cout << std::endl
    << "Bundle Adjustment of cameras [R|t|f] and Structure : \n"
    << " Initial RMSE : " << residual_before << "\n"
    << " Final RMSE : " << residual_after << std::endl;

  // If no error, get back refined parameters
  if (summary.IsSolutionUsable())
  {
    // Get back 3D points
    size_t cpt = 0;
    for (std::vector<Vec3>::iterator iter = vec_3d_points.begin();
      iter != vec_3d_points.end(); ++iter, ++cpt)
    {
      const double * pt = ba_problem.mutable_points() + cpt*3;
      Vec3 & point_3d = *iter;
      point_3d = Vec3(pt[0], pt[1], pt[2]);
    }
    // Get back camera
    for (cpt = 0; cpt < nviews; ++cpt)
    {
      const double * cam = ba_problem.mutable_cameras() + cpt*7;
      Mat3 R;
      // angle axis to rotation matrix
      ceres::AngleAxisToRotationMatrix(cam, R.data());
      Vec3 t(cam[3], cam[4], cam[5]);
      double focal = cam[6];

      // Update the camera
      PinholeCamera & sCam = vec_cam[cpt];
      Mat3 camera_matrix = sCam.camera_matrix_;
      camera_matrix(0,0) = camera_matrix(1,1) = focal;
      std::cout << "Refined focal[" << cpt << "]: " << focal << std::endl;
      sCam = PinholeCamera(camera_matrix, R, t);
    }
  }
}

/// Perform a Bundle Adjustment: Refine the cameras [R|t]
///  a common [focal] and the structure
void do_bundle_adjustment_common_focal(
  PinholeCamera & left_camera,
  PinholeCamera & right_camera,
  const Mat & left_points,
  const Mat & right_points,
  const std::vector<size_t> & vec_inliers,
  std::vector<Vec3> & vec_3d_points)
{
  int nCameraMotion = 2;
  int nCameraIntrinsic = 1;
  int n3Dpoints = vec_inliers.size();

  // Setup a BA problem
  BA_Problem_data_camMotionAndIntrinsic<6,1> ba_problem;

  // Configure the size of the problem
  ba_problem.num_cameras_ = nCameraMotion;
  ba_problem.num_intrinsic_ = nCameraIntrinsic;
  ba_problem.num_points_ = n3Dpoints;
  ba_problem.num_observations_ = nCameraMotion * n3Dpoints;

  ba_problem.point_index_.reserve(ba_problem.num_observations_);
  ba_problem.camera_index_extrinsic.reserve(ba_problem.num_observations_);
  ba_problem.camera_index_intrinsic.reserve(ba_problem.num_observations_);
  ba_problem.observations_.reserve(2 * ba_problem.num_observations_);

  ba_problem.num_parameters_ =
    6 * ba_problem.num_cameras_ + ba_problem.num_intrinsic_ + 3 * ba_problem.num_points_;
  ba_problem.parameters_.reserve(ba_problem.num_parameters_);

  // fill it with data (For each 3D point setup the tracks : the 2D visbility)
  // The two camera share the same intrinsic
  PinholeCamera vec_cam[2] = {left_camera, right_camera};
  for (int i = 0; i < n3Dpoints; ++i) {
    // Collect the image of point i in each frame (left_points, right_points).
    const Vec2 & xL_ = left_points.col(vec_inliers[i]);
    const Vec2 & xR_ = right_points.col(vec_inliers[i]);

    // Left 2D observations
    double ppx = vec_cam[0].camera_matrix_(0,2), ppy = vec_cam[0].camera_matrix_(1,2);
    ba_problem.camera_index_extrinsic.push_back(0);
    ba_problem.camera_index_intrinsic.push_back(0);
    ba_problem.point_index_.push_back(i);
    ba_problem.observations_.push_back( xL_(0) - ppx);
    ba_problem.observations_.push_back( xL_(1) - ppy);

    // Right 2D observations
    ppx = vec_cam[1].camera_matrix_(0,2);
    ppy = vec_cam[1].camera_matrix_(1,2);
    ba_problem.camera_index_extrinsic.push_back(1);
    ba_problem.camera_index_intrinsic.push_back(0); // same intrinsic group
    ba_problem.point_index_.push_back(i);
    ba_problem.observations_.push_back( xR_(0) - ppx);
    ba_problem.observations_.push_back( xR_(1) - ppy);
  }

  // Add camera extrinsics [R,t]
  for (int j = 0; j < nCameraMotion; ++j) {
    // Rotation matrix to angle axis
    std::vector<double> angleAxis(3);
    ceres::RotationMatrixToAngleAxis((const double*)vec_cam[j].rotation_matrix_.data(), &angleAxis[0]);
    // translation
    Vec3 t = vec_cam[j].translation_vector_;
    ba_problem.parameters_.push_back(angleAxis[0]);
    ba_problem.parameters_.push_back(angleAxis[1]);
    ba_problem.parameters_.push_back(angleAxis[2]);
    ba_problem.parameters_.push_back(t[0]);
    ba_problem.parameters_.push_back(t[1]);
    ba_problem.parameters_.push_back(t[2]);
  }
  // Add camera intrinsic (focal)
  double focal = (vec_cam[0].camera_matrix_(0,0) + vec_cam[0].camera_matrix_(1,1)
     + vec_cam[1].camera_matrix_(1,1) + vec_cam[1].camera_matrix_(0,0)) / 4.0;
  // Setup the focal in the ba_problem
  ba_problem.parameters_.push_back(focal);

  // Add 3D points coordinates parameters
  for (int i = 0; i < n3Dpoints; ++i) {
    Vec3 point_3d = vec_3d_points[i];
    ba_problem.parameters_.push_back(point_3d[0]);
    ba_problem.parameters_.push_back(point_3d[1]);
    ba_problem.parameters_.push_back(point_3d[2]);
  }

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < ba_problem.num_observations(); ++i) {

    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ErrorFuncRefineCamera3DPointsFocal, 2, 1, 6, 3>(
            new ErrorFuncRefineCamera3DPointsFocal(
                & ba_problem.observations()[2 * i]));

    problem.AddResidualBlock(cost_function,
                             NULL, // squared loss
                             ba_problem.mutable_camera_intrisic_for_observation(i),
                             ba_problem.mutable_camera_extrinsic_for_observation(i),
                             ba_problem.mutable_point_for_observation(i));
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  else
    if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
      options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
    else
    {
      // No sparse backend for Ceres.
      // Use dense solving
      options.linear_solver_type = ceres::DENSE_SCHUR;
    }
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  double residual_before = std::sqrt( summary.initial_cost / (ba_problem.num_observations_*2.));
  double residual_after = std::sqrt( summary.final_cost / (ba_problem.num_observations_*2.));

  std::cout << std::endl
    << "Bundle Adjustment of cameras [R|t], shared [f] and Structure : \n"
    << " Initial RMSE : " << residual_before << "\n"
    << " Final RMSE : " << residual_after << "\n"
    << "Initial focal : " << focal << std::endl;

  // If no error, get back refined parameters
  if (summary.IsSolutionUsable())
  {
    // Get back 3D points
    size_t cpt = 0;
    for (std::vector<Vec3>::iterator iter = vec_3d_points.begin();
      iter != vec_3d_points.end(); ++iter, ++cpt)
    {
      const double * pt = ba_problem.mutable_points() + cpt*3;
      Vec3 & point_3d = *iter;
      point_3d = Vec3(pt[0], pt[1], pt[2]);
    }
    // Get back camera
    for (cpt = 0; cpt < nCameraMotion; ++cpt)
    {
      const double * cam = ba_problem.mutable_cameras_extrinsic() + cpt*6;
      Mat3 R;
      // angle axis to rotation matrix
      ceres::AngleAxisToRotationMatrix(cam, R.data());
      Vec3 t(cam[3], cam[4], cam[5]);

      // Update the camera
      PinholeCamera & sCam = vec_cam[cpt];
      Mat3 camera_matrix = sCam.camera_matrix_;
      double focal = *ba_problem.mutable_cameras_intrinsic();
      std::cout << "Refined focal[" << cpt << "]: " << focal << std::endl;
      camera_matrix(0,0) = camera_matrix(1,1) = focal;
      sCam = PinholeCamera(camera_matrix, R, t);
    }
  }
}

/// Perform a Bundle Adjustment: Refine the cameras [R|t]
///  and common intrinsics [focal,ppx,ppy,k1,k2,k3] and the structure
void do_bundle_adjustment_common_intrinsics(
  PinholeCamera & left_camera,
  PinholeCamera & right_camera,
  const Mat & left_points,
  const Mat & right_points,
  const std::vector<size_t> & vec_inliers,
  std::vector<Vec3> & vec_3d_points)
{
  int nCameraMotion = 2;
  int nCameraIntrinsic = 1;
  int n3Dpoints = vec_inliers.size();

  // Setup a BA problem
  BA_Problem_data_camMotionAndIntrinsic<6, 6> ba_problem;

  // Configure the size of the problem
  ba_problem.num_cameras_ = nCameraMotion;
  ba_problem.num_intrinsic_ = nCameraIntrinsic;
  ba_problem.num_points_ = n3Dpoints;
  ba_problem.num_observations_ = nCameraMotion * n3Dpoints;

  ba_problem.point_index_.reserve(ba_problem.num_observations_);
  ba_problem.camera_index_extrinsic.reserve(ba_problem.num_observations_);
  ba_problem.camera_index_intrinsic.reserve(ba_problem.num_observations_);
  ba_problem.observations_.reserve(2 * ba_problem.num_observations_);

  ba_problem.num_parameters_ =
    6 * ba_problem.num_cameras_ + 6 * ba_problem.num_intrinsic_ + 3 * ba_problem.num_points_;
  ba_problem.parameters_.reserve(ba_problem.num_parameters_);

  // fill it with data (For each 3D point setup the tracks : the 2D visbility)
  // The two camera share the same intrinsic
  PinholeCamera vec_cam[2] = {left_camera, right_camera};
  for (int i = 0; i < n3Dpoints; ++i) {
    // Collect the image of point i in each frame (left_points, right_points).
    const Vec2 & xL_ = left_points.col(vec_inliers[i]);
    const Vec2 & xR_ = right_points.col(vec_inliers[i]);

    // Left 2D observations
    ba_problem.camera_index_extrinsic.push_back(0);
    ba_problem.camera_index_intrinsic.push_back(0);
    ba_problem.point_index_.push_back(i);
    ba_problem.observations_.push_back(xL_(0));
    ba_problem.observations_.push_back(xL_(1));

    // Right 2D observations
    ba_problem.camera_index_extrinsic.push_back(1);
    ba_problem.camera_index_intrinsic.push_back(0); // same intrinsic group
    ba_problem.point_index_.push_back(i);
    ba_problem.observations_.push_back(xR_(0));
    ba_problem.observations_.push_back(xR_(1));
  }

  // Add camera extrinsics [R,t]
  for (int j = 0; j < nCameraMotion; ++j) {
    // Rotation matrix to angle axis
    std::vector<double> angleAxis(3);
    ceres::RotationMatrixToAngleAxis((const double*)vec_cam[j].rotation_matrix_.data(), &angleAxis[0]);
    // translation
    Vec3 t = vec_cam[j].translation_vector_;
    ba_problem.parameters_.push_back(angleAxis[0]);
    ba_problem.parameters_.push_back(angleAxis[1]);
    ba_problem.parameters_.push_back(angleAxis[2]);
    ba_problem.parameters_.push_back(t[0]);
    ba_problem.parameters_.push_back(t[1]);
    ba_problem.parameters_.push_back(t[2]);
  }

  // Add camera intrinsics (focal, ppx, ppy, k1, k2, k3)
  {
    double focal = (vec_cam[0].camera_matrix_(0,0) + vec_cam[0].camera_matrix_(1,1)
      + vec_cam[1].camera_matrix_(1,1) + vec_cam[1].camera_matrix_(0,0)) / 4.0;
    double ppx = (vec_cam[0].camera_matrix_(0,2) + vec_cam[1].camera_matrix_(0,2)) / 2.0;
    double ppy = (vec_cam[0].camera_matrix_(1,2) + vec_cam[1].camera_matrix_(1,2)) / 2.0;
    double k1 = 0.0, k2 = 0.0, k3 = 0.0;

    // Setup intrinsics in the ba_problem
    ba_problem.parameters_.push_back(focal);
    ba_problem.parameters_.push_back(ppx);
    ba_problem.parameters_.push_back(ppy);
    ba_problem.parameters_.push_back(k1);
    ba_problem.parameters_.push_back(k2);
    ba_problem.parameters_.push_back(k3);
  }


  // Add 3D points coordinates parameters
  for (int i = 0; i < n3Dpoints; ++i) {
    Vec3 point_3d = vec_3d_points[i];
    double * ptr3D = ba_problem.mutable_points() + i * 3;
    ba_problem.parameters_.push_back(point_3d[0]);
    ba_problem.parameters_.push_back(point_3d[1]);
    ba_problem.parameters_.push_back(point_3d[2]);
  }

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < ba_problem.num_observations(); ++i) {

    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ErrorFunc_Refine_Camera_3DPoints, 2, 6, 6, 3>(
            new ErrorFunc_Refine_Camera_3DPoints(
                & ba_problem.observations()[2 * i + 0]));

    problem.AddResidualBlock(cost_function,
                             NULL, // squared loss
                             ba_problem.mutable_camera_intrisic_for_observation(i),
                             ba_problem.mutable_camera_extrinsic_for_observation(i),
                             ba_problem.mutable_point_for_observation(i));
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  else
    if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
      options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
    else
    {
      // No sparse backend for Ceres.
      // Use dense solving
      options.linear_solver_type = ceres::DENSE_SCHUR;
    }
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport() << std::endl;

  double residual_before = std::sqrt( summary.initial_cost / (ba_problem.num_observations_*2.));
  double residual_after = std::sqrt( summary.final_cost / (ba_problem.num_observations_*2.));

  std::cout << std::endl
    << "Bundle Adjustment of struture [X], cameras extrinsics [R|t],"
    << " and shared intrinsics [f,ppx,ppy,k1,k2,k3]: \n"
    << " Initial RMSE : " << residual_before << "\n"
    << " Final RMSE : " << residual_after << "\n"
    << "Refined intrinsics : " << std::endl;

  // If no error, get back refined parameters
  if (summary.IsSolutionUsable())
  {
    // Get back 3D points
    size_t cpt = 0;
    for (std::vector<Vec3>::iterator iter = vec_3d_points.begin();
      iter != vec_3d_points.end(); ++iter, ++cpt)
    {
      const double * pt = ba_problem.mutable_points() + cpt*3;
      Vec3 & point_3d = *iter;
      point_3d = Vec3(pt[0], pt[1], pt[2]);
    }
    // Get back camera
    for (cpt = 0; cpt < nCameraMotion; ++cpt)
    {
      const double * cam = ba_problem.mutable_cameras_extrinsic() + cpt*6;
      Mat3 R;
      // angle axis to rotation matrix
      ceres::AngleAxisToRotationMatrix(cam, R.data());
      Vec3 t(cam[3], cam[4], cam[5]);

      // Update the camera
      PinholeCamera & sCam = vec_cam[cpt];
      const double * camIntrinsics = ba_problem.mutable_cameras_intrinsic();
      std::cout << " for camera Idx=[" << cpt << "]: " << std::endl
        << "\t focal: " << camIntrinsics[OFFSET_FOCAL_LENGTH] << std::endl
        << "\t ppx: " << camIntrinsics[OFFSET_PRINCIPAL_POINT_X] << std::endl
        << "\t ppy: " << camIntrinsics[OFFSET_PRINCIPAL_POINT_Y] << std::endl
        << "\t k1: " << camIntrinsics[OFFSET_K1] << std::endl
        << "\t k2: " << camIntrinsics[OFFSET_K2] << std::endl
        << "\t k3: " << camIntrinsics[OFFSET_K3] << std::endl
        << "\t initial : focal: " << sCam.camera_matrix_(0,0) << ", ppx: " << sCam.camera_matrix_(0,2)
        << ", ppy: " << sCam.camera_matrix_(1,2) <<std::endl;
      Mat3 camera_matrix = sCam.camera_matrix_;
      camera_matrix(0,0) = camera_matrix(1,1) = camIntrinsics[OFFSET_FOCAL_LENGTH];
      sCam.camera_matrix_(0,2) = camIntrinsics[OFFSET_PRINCIPAL_POINT_X];
      sCam.camera_matrix_(1,2) = camIntrinsics[OFFSET_PRINCIPAL_POINT_Y];
      sCam = PinholeCamera(camera_matrix, R, t);
    }
  }
}
