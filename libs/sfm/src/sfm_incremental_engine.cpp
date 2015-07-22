#include "sfm_precomp.h"
#include <numeric>
#include <iomanip>
#include <algorithm>
#include <functional>
#include <sstream>

#include "fblib/image/image.h"
#include "fblib/feature/indexed_match_utils.h"
#include "fblib/feature/indexed_match.h"
#include "fblib/sfm/pinhole_brown_rt_ceres_functor.h"
#include "fblib/sfm/problem_data_container.h"
#include "fblib/sfm/sfm_incremental_engine.h"
#include "fblib/sfm/sfm_robust.h"

#include "fblib/tracking/tracks.h"

#include "fblib/utils/file_system.h"
#include "fblib/utils/svg_drawer.h"
#include "fblib/utils/stl_map.h"
#include "fblib/utils/indexed_sort.h"
#include "fblib/utils/notify.h"

using namespace fblib::tracking;

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

namespace fblib{
	namespace sfm{

		typedef ScalePointFeature FeatureT;
		typedef std::vector<FeatureT> featsT;

		IncrementalReconstructionEngine::IncrementalReconstructionEngine(const std::string &image_path,
			const std::string &matches_path, const std::string &out_dir, bool is_html_report)
			: ReconstructionEngine(image_path, matches_path, out_dir),
			initial_pair_(std::make_pair<size_t, size_t>(0, 0)),
			is_refine_point_and_distortion_(true),
			is_use_bundle_adjustment_(true)
		{
			is_html_report_ = is_html_report;
			if (!fblib::utils::folder_exists(out_dir)) {
				fblib::utils::folder_create(out_dir);
			}
			if (is_html_report_)
			{
				html_doc_stream_ = std::auto_ptr<HtmlDocumentStream>(
					new HtmlDocumentStream("fblib A Contrario Incremental SFM report."));

				html_doc_stream_->pushInfo(
					htmlMarkup("h1", "fblib A Contrario Incremental SFM report."));

				html_doc_stream_->pushInfo(
					htmlMarkup("h1", std::string("Current directory: ") +
					image_path));
				html_doc_stream_->pushInfo("<hr>");
			}
		}

		IncrementalReconstructionEngine::~IncrementalReconstructionEngine()
		{
			ofstream htmlFileStream(string(fblib::utils::folder_append_separator(out_dir_) +
				"Reconstruction_Report.html").c_str());
			htmlFileStream << html_doc_stream_->getDoc();
		}

		void PauseProcess()
		{
			unsigned char i;
			FBLIB_INFO << "\nPause : type key and press enter: ";
			cin >> i;
		}

		bool IncrementalReconstructionEngine::Process()
		{
			// 导入数据
			if (!ReadInputData())
				return false;

			// 增量式重建
			std::pair<size_t, size_t> initial_pair_index;
			if (!InitialPairChoice(initial_pair_index))
				return false;

			// Initial pair Essential Matrix and [R|t] estimation.
			if (!MakeInitialPair3D(initial_pair_index))
				return false;

			BundleAdjustment(); // Adjust 3D point and camera parameters.

			size_t round = 0;
			bool bImageAdded = false;
			// Compute robust Resection of remaining image
			std::vector<size_t> vec_possible_resection_indexes;
			while (FindImagesWithPossibleResection(vec_possible_resection_indexes))
			{
				if (Resection(vec_possible_resection_indexes))
				{
					std::ostringstream os;
					os << std::setw(8) << std::setfill('0') << round << "_Resection";
					reconstructor_data_.exportToPlyFile(fblib::utils::create_filespec(out_dir_, os.str(), ".ply"));
					bImageAdded = true;
				}
				++round;
				if (bImageAdded && is_use_bundle_adjustment_)
				{
					// Perform BA until all point are under the given precision
					do
					{
						BundleAdjustment();
					} while (badTrackRejector(4.0) != 0);
				}
			}

			//-- Reconstruction done.
			//-- Display some statistics
			FBLIB_INFO << "\n\n-------------------------------" << "\n"
				<< "-- Structure from Motion (statistics):\n"
				<< "-- #Camera calibrated: " << reconstructor_data_.map_Camera.size()
				<< " from " << camera_image_names_.size() << " input images.\n"
				<< "-- #Tracks, #3D points: " << reconstructor_data_.map_3DPoints.size() << "\n"
				<< "-------------------------------" << "\n";

			Histogram<double> h;
			ComputeResidualsHistogram(&h);
			FBLIB_INFO << "\nHistogram of residuals:" << h.ToString() << std::endl;

			if (is_html_report_)
			{
				std::ostringstream os;
				os << "Structure from Motion process finished.";
				html_doc_stream_->pushInfo("<hr>");
				html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

				os.str("");
				os << "-------------------------------" << "<br>"
					<< "-- Structure from Motion (statistics):<br>"
					<< "-- #Camera calibrated: " << reconstructor_data_.map_Camera.size()
					<< " from " << camera_image_names_.size() << " input images.<br>"
					<< "-- #Tracks, #3D points: " << reconstructor_data_.map_3DPoints.size() << "<br>"
					<< "-------------------------------" << "<br>";
				html_doc_stream_->pushInfo(os.str());

				html_doc_stream_->pushInfo(htmlMarkup("h2", "Histogram of reprojection-residuals"));

				std::vector<double> xBin = h.GetXbinsValue();
				std::pair< std::pair<double, double>, std::pair<double, double> > range;
				range = autoJSXGraphViewport<double>(xBin, h.GetHist());

				JSXGraphWrapper jsxGraph;
				jsxGraph.init("3DtoImageResiduals", 600, 300);
				jsxGraph.addXYChart(xBin, h.GetHist(), "line,point");
				jsxGraph.UnsuspendUpdate();
				jsxGraph.setViewport(range);
				jsxGraph.close();
				html_doc_stream_->pushInfo(jsxGraph.toStr());
			}
			return true;
		}

		bool IncrementalReconstructionEngine::ReadInputData()
		{
			if (!fblib::utils::is_folder(image_path_) ||
				!fblib::utils::is_folder(matches_path_) ||
				!fblib::utils::is_folder(out_dir_))
			{
				std::cerr << std::endl
					<< "One of the required directory is not a valid directory" << std::endl;
				return false;
			}

			std::string file_lists = fblib::utils::create_filespec(matches_path_, "lists", "txt");
			std::string computed_matches_file = fblib::utils::create_filespec(matches_path_, "matches.f", "txt");
			if (!fblib::utils::is_file(file_lists) ||
				!fblib::utils::is_file(computed_matches_file))
			{
				std::cerr << std::endl
					<< "One of the input required file is not a present (lists.txt, matches.f.txt)" << std::endl;
				return false;
			}

			// a、读取图像名
			if (!fblib::feature::LoadImageList(file_lists, camera_image_names_,
				vec_intrinsic_groups_
				))
			{
				std::cerr << "\nEmpty image list." << std::endl;
				return false;
			}
			else
			{
				// 找到图像对应的内参
				for (std::vector<fblib::feature::CameraInfo>::const_iterator iter = camera_image_names_.begin();
					iter != camera_image_names_.end(); ++iter)
				{
					const fblib::feature::CameraInfo &camera_info = *iter;
					// 得到索引
					size_t idx = std::distance((std::vector<fblib::feature::CameraInfo>::const_iterator)camera_image_names_.begin(), iter);
					set_remaining_image_id_.insert(idx);
					map_intrinsic_id_per_image_id_[idx] = camera_info.intrinsic_id;
				}
			}

			// b. 读取对应匹配图片
			if (!PairedIndexedMatchImport(computed_matches_file, map_matches_fundamental_)) {
				std::cerr << "Unable to read the fundamental matrix matches" << std::endl;
				return false;
			}

			// c. Compute tracks from matches
			TracksBuilder tracks_builder;

			{
				FBLIB_INFO << std::endl << "Track building" << std::endl;
				tracks_builder.Build(map_matches_fundamental_);
				FBLIB_INFO << std::endl << "Track filtering" << std::endl;
				tracks_builder.Filter();
				FBLIB_INFO << std::endl << "Track filtering : min occurence" << std::endl;
				tracks_builder.FilterPairWiseMinimumMatches(20);
				FBLIB_INFO << std::endl << "Track export to internal struct" << std::endl;
				//-- Build tracks with STL compliant type :
				tracks_builder.ExportToSTL(map_tracks_);

				FBLIB_INFO << std::endl << "Track stats" << std::endl;
				{
					std::ostringstream osTrack;
					//-- Display stats :
					//    - number of images
					//    - number of tracks
					std::set<size_t> set_imagesId;
					TracksUtilsMap::ImageIdInTracks(map_tracks_, set_imagesId);
					osTrack << "------------------" << "\n"
						<< "-- Tracks Stats --" << "\n"
						<< " Tracks number: " << tracks_builder.NbTracks() << "\n"
						<< " Images Id: " << "\n";
					std::copy(set_imagesId.begin(),
						set_imagesId.end(),
						std::ostream_iterator<size_t>(osTrack, ", "));
					osTrack << "\n------------------" << "\n";

					std::map<size_t, size_t> map_Occurence_TrackLength;
					TracksUtilsMap::TracksLength(map_tracks_, map_Occurence_TrackLength);
					osTrack << "TrackLength, Occurrence" << "\n";
					for (std::map<size_t, size_t>::const_iterator iter = map_Occurence_TrackLength.begin();
						iter != map_Occurence_TrackLength.end(); ++iter)  {
						osTrack << "\t" << iter->first << "\t" << iter->second << "\n";
					}
					osTrack << "\n";
					FBLIB_INFO << osTrack.str();
				}
			}

			// Read features:
			for (size_t i = 0; i < camera_image_names_.size(); ++i)  {
				const size_t camIndex = i;
				if (!LoadFeatsFromFile(
					fblib::utils::create_filespec(matches_path_, fblib::utils::basename_part(camera_image_names_[camIndex].image_name), ".feat"),
					map_features_[camIndex])) {
					std::cerr << "Bad reading of feature files" << std::endl;
					return false;
				}
			}

			if (is_html_report_)
			{
				html_doc_stream_->pushInfo("Dataset info:");
				html_doc_stream_->pushInfo("Number of images: " +
					toString(camera_image_names_.size()) + "<br>");

				std::ostringstream osTrack;
				{
					//-- Display stats :
					//    - number of images
					//    - number of tracks
					std::set<size_t> set_imagesId;
					TracksUtilsMap::ImageIdInTracks(map_tracks_, set_imagesId);
					osTrack << "------------------" << "<br>"
						<< "-- Tracks Stats --" << "<br>"
						<< " Tracks number: " << tracks_builder.NbTracks() << "<br>"
						<< " Images Id: " << "<br>";
					std::copy(set_imagesId.begin(),
						set_imagesId.end(),
						std::ostream_iterator<size_t>(osTrack, ", "));
					osTrack << "<br>------------------" << "<br>";

					std::map<size_t, size_t> map_Occurence_TrackLength;
					TracksUtilsMap::TracksLength(map_tracks_, map_Occurence_TrackLength);
					osTrack << "<table> <tr> <td>TrackLength, </td> <td>Occurrence<td> </tr>";
					for (std::map<size_t, size_t>::const_iterator iter = map_Occurence_TrackLength.begin();
						iter != map_Occurence_TrackLength.end(); ++iter)
					{
						osTrack << "<tr><td>" << iter->first << "</td><td>"
							<< iter->second << "</td></tr>";
					}
					osTrack << "</table>";
				}

				html_doc_stream_->pushInfo(osTrack.str());
				html_doc_stream_->pushInfo("<br>");

				html_doc_stream_->pushInfo("<hr>");
			}
			return true;
		}

		/**
		* \brief	选择最好的初始匹配对
		*
		* \param [in,out]	initial_pair_index	返回最好的初始匹配对
		*
		* \return	true if it succeeds, false if it fails.
		*/
		bool IncrementalReconstructionEngine::InitialPairChoice(std::pair<size_t, size_t> & initial_pair_index)
		{
			if (initial_pair_ != std::make_pair<size_t, size_t>(0, 0))
			{
				initial_pair_index = initial_pair_;
			}
			else
			{
				FBLIB_INFO << std::endl
					<< "---------------------------------------------------\n"
					<< "IncrementalReconstructionEngine::InitialPairChoice\n"
					<< "---------------------------------------------------\n"
					<< " The best fundamental_matrix matrix validated pairs are displayed\n"
					<< " Choose one pair manually by typing the two integer indexes\n"
					<< "---------------------------------------------------\n"
					<< std::endl;

				// Display to the user the 10 top Fundamental matches pair
				std::vector< size_t > vec_NbMatchesPerPair;
				for (fblib::feature::PairWiseMatches::const_iterator
					iter = map_matches_fundamental_.begin();
					iter != map_matches_fundamental_.end(); ++iter)
				{
					vec_NbMatchesPerPair.push_back(iter->second.size());
				}
				// sort in descending order
				std::vector< fblib::utils::SortIndexPacketDescend< size_t, size_t> > packet_vec(vec_NbMatchesPerPair.size());
				fblib::utils::SortIndexHelper(packet_vec, &vec_NbMatchesPerPair[0], std::min((size_t)10, map_matches_fundamental_.size()));

				for (size_t i = 0; i < std::min((size_t)10, map_matches_fundamental_.size()); ++i) {
					size_t index = packet_vec[i].index;
					fblib::feature::PairWiseMatches::const_iterator iter = map_matches_fundamental_.begin();
					std::advance(iter, index);
					FBLIB_INFO << "(" << iter->first.first << "," << iter->first.second << ")\t\t"
						<< iter->second.size() << " matches" << std::endl;
				}

				// Manual choice of the initial pair
				FBLIB_INFO << std::endl << " type INITIAL pair Indexes: X enter Y enter\n";
				int val, val2;
				if (std::cin >> val && std::cin >> val2) {
					initial_pair_index.first = val;
					initial_pair_index.second = val2;
				}
			}

			FBLIB_INFO << "\nPutative starting pair is: (" << initial_pair_index.first
				<< "," << initial_pair_index.second << ")" << std::endl;

			// Check validity of the initial pair indices:
			if (map_features_.find(initial_pair_index.first) == map_features_.end() ||
				map_features_.find(initial_pair_index.second) == map_features_.end())
			{
				std::cerr << "At least one of the initial pair indices is invalid."
					<< std::endl;
				return false;
			}
			return true;
		}

		bool IncrementalReconstructionEngine::MakeInitialPair3D(const std::pair<size_t, size_t> &initial_pair)
		{
			// Compute robust Essential matrix for ImageId [I,J]
			// use min max to have I < J
			const size_t I = std::min(initial_pair.first, initial_pair.second);
			const size_t J = std::max(initial_pair.first, initial_pair.second);

			// Check validity of the initial pair indices:
			if (map_features_.find(I) == map_features_.end() ||
				map_features_.find(J) == map_features_.end())
			{
				std::cerr << "At least one of the initial pair indices is invalid."
					<< std::endl;
				return false;
			}

			// a.coords Get common tracks between the two images
			fblib::tracking::MapTracks map_tracksCommon;
			std::set<size_t> set_image_index;
			set_image_index.insert(I);
			set_image_index.insert(J);
			TracksUtilsMap::GetTracksInImages(set_image_index, map_tracks_, map_tracksCommon);

			// b. Get corresponding features
			std::vector<ScalePointFeature> & vec_featI = map_features_[I];
			std::vector<ScalePointFeature> & vec_featJ = map_features_[J];

			//-- Copy point to array in order to estimate essential matrix:
			const size_t n = map_tracksCommon.size();
			Mat x1(2, n), x2(2, n);
			size_t cptIndex = 0;
			for (fblib::tracking::MapTracks::const_iterator
				iterT = map_tracksCommon.begin();
				iterT != map_tracksCommon.end();
			++iterT)
			{
				//Get corresponding point
				tracking::SubmapTrack::const_iterator iter = iterT->second.begin();
				const size_t i = iter->second;
				const size_t j = (++iter)->second;
				const ScalePointFeature & left_feature = vec_featI[i];
				const ScalePointFeature & right_feature = vec_featJ[j];

				x1.col(cptIndex) = left_feature.coords().cast<double>();
				x2.col(cptIndex) = right_feature.coords().cast<double>();
				++cptIndex;
			}

			Mat3 E = Mat3::Identity();
			std::vector<size_t> vec_inliers;
			double errorMax = std::numeric_limits<double>::max();

			const fblib::feature::IntrinsicCameraInfo & intrinsicCamI = vec_intrinsic_groups_[camera_image_names_[I].intrinsic_id];
			const fblib::feature::IntrinsicCameraInfo & intrinsicCamJ = vec_intrinsic_groups_[camera_image_names_[J].intrinsic_id];

			if (!intrinsicCamI.is_known_intrinsic ||
				!intrinsicCamJ.is_known_intrinsic)
			{
				std::cerr << "The selected image pair doesn't have intrinsic parameters, we cannot estimate an initial reconstruction from the two provided index: [I,J]=[" << I << "," << J << "]" << std::endl;
				return false;
			}

			if (!robustEssential(intrinsicCamI.camera_matrix, intrinsicCamJ.camera_matrix,
				x1, x2,
				&E, &vec_inliers,
				std::make_pair(intrinsicCamI.width, intrinsicCamI.height),
				std::make_pair(intrinsicCamJ.width, intrinsicCamJ.height),
				&errorMax))
			{
				std::cerr << " /!\\ Robust estimation failed to compute E for the initial pair" << std::endl;
				return false;
			}

			FBLIB_INFO << std::endl
				<< "-- Robust Essential Matrix estimation " << std::endl
				<< "--  #tentative/#inliers: " << x1.cols() << " / " << vec_inliers.size() << std::endl
				<< "--  Threshold: " << errorMax << std::endl;

			//--> Estimate the best possible Rotation/Translation from E
			Mat3 RJ;
			Vec3 tJ;
			if (!EstimateRtFromE(intrinsicCamI.camera_matrix, intrinsicCamJ.camera_matrix,
				x1, x2, E, vec_inliers,
				&RJ, &tJ))
			{
				FBLIB_INFO << " /!\\ Failed to compute initial R|t for the initial pair" << std::endl;
				return false;
			}
			FBLIB_INFO << std::endl
				<< "-- Rotation|Translation matrices: --" << std::endl
				<< RJ << std::endl << std::endl << tJ << std::endl;

			//-> Triangulate the common tracks
			//--> Triangulate the point

			BrownPinholeCamera camI(intrinsicCamI.focal, intrinsicCamI.camera_matrix(0, 2), intrinsicCamI.camera_matrix(1, 2), Mat3::Identity(), Vec3::Zero());
			BrownPinholeCamera camJ(intrinsicCamJ.focal, intrinsicCamJ.camera_matrix(0, 2), intrinsicCamJ.camera_matrix(1, 2), RJ, tJ);

			std::vector<IndexedMatch> vec_index;
			for (fblib::tracking::MapTracks::const_iterator
				iterT = map_tracksCommon.begin();
				iterT != map_tracksCommon.end();
			++iterT)
			{
				tracking::SubmapTrack::const_iterator iter = iterT->second.begin();
				tracking::SubmapTrack::const_iterator iter2 = iterT->second.begin();
				std::advance(iter2, 1);
				vec_index.push_back(IndexedMatch(iter->second, iter2->second));
			}
			std::vector<Vec3> vec_3dPoint;
			std::vector<double> vec_triangulationResidual;

			triangulate2View_Vector(camI.projection_matrix_, camJ.projection_matrix_,
				vec_featI, vec_featJ,
				vec_index, &vec_3dPoint, &vec_triangulationResidual);

			//- Add reconstructed point to the reconstruction data
			//- Write corresponding that the track have a corresponding 3D point
			cptIndex = 0;
			for (fblib::tracking::MapTracks::const_iterator
				iterT = map_tracksCommon.begin();
				iterT != map_tracksCommon.end();
			++iterT, cptIndex++)
			{
				size_t trackId = iterT->first;
				const Vec2 xI = vec_featI[vec_index[cptIndex]._i].coords().cast<double>();
				const Vec2 xJ = vec_featJ[vec_index[cptIndex]._j].coords().cast<double>();

				if (find(vec_inliers.begin(), vec_inliers.end(), cptIndex) != vec_inliers.end())
				{
					//-- Depth
					//-- Residuals
					//-- Angle between rays ?
					if (camI.Depth(vec_3dPoint[cptIndex]) > 0
						&& camJ.Depth(vec_3dPoint[cptIndex]) > 0)  {
						double angle = BrownPinholeCamera::AngleBetweenRay(camI, camJ, xI, xJ);
						if (angle > 2.)  {
							reconstructor_data_.map_3DPoints[trackId] = vec_3dPoint[cptIndex];
							reconstructor_data_.set_trackId.insert(trackId);
							map_reconstructed_[trackId].insert(make_pair(I, vec_index[cptIndex]._i));
							map_reconstructed_[trackId].insert(make_pair(J, vec_index[cptIndex]._j));
						}
					}
				}
				else  {
					//Remove this track entry with ImageIndexes
					map_tracks_[trackId].erase(I);
					map_tracks_[trackId].erase(J);
					if (map_tracks_[trackId].size() < 2)  {
						map_tracks_[trackId].clear();
						map_tracks_.erase(trackId);
					}
				}
			}

			FBLIB_INFO << "--#Triangulated 3D points: " << vec_inliers.size() << "\n";
			FBLIB_INFO << "--#Triangulated 3D points under threshold: " << reconstructor_data_.map_3DPoints.size() << "\n";
			FBLIB_INFO << "--#Putative correspondences: " << x1.cols() << "\n";

			set_remaining_image_id_.erase(I);
			set_remaining_image_id_.erase(J);

			if (!reconstructor_data_.map_3DPoints.empty())
			{
				// Add information related to the View (I,J) to the reconstruction data
				reconstructor_data_.set_imagedId.insert(I);
				reconstructor_data_.set_imagedId.insert(J);

				reconstructor_data_.map_Camera.insert(std::make_pair(I, camI));
				reconstructor_data_.map_Camera.insert(std::make_pair(J, camJ));

				// Add the images to the reconstructed intrinsic group
				map_images_id_per_intrinsic_group_[map_intrinsic_id_per_image_id_[I]].push_back(I);
				map_images_id_per_intrinsic_group_[map_intrinsic_id_per_image_id_[J]].push_back(J);
				// Initialize the first intrinsics groups:
				Vec6 & intrinsicI = map_intrinsics_per_group_[map_intrinsic_id_per_image_id_[I]];
				intrinsicI << camI._f, camI._ppx, camI._ppy, camI._k1, camI._k2, camI._k3;
				Vec6 & intrinsicJ = map_intrinsics_per_group_[map_intrinsic_id_per_image_id_[J]];
				intrinsicJ << camJ._f, camJ._ppx, camJ._ppy, camJ._k1, camJ._k2, camJ._k3;

				map_ac_threshold_.insert(std::make_pair(I, errorMax));
				map_ac_threshold_.insert(std::make_pair(J, errorMax));

				vec_added_order_.push_back(I);
				vec_added_order_.push_back(J);
			}


			reconstructor_data_.exportToPlyFile(fblib::utils::create_filespec(out_dir_, "sceneStart", "ply"));

			Histogram<double> histoResiduals;
			FBLIB_INFO << std::endl
				<< "=========================\n"
				<< " MSE Residual InitialPair Inlier: " << ComputeResidualsHistogram(&histoResiduals) << "\n"
				<< "=========================" << std::endl;

			if (is_html_report_)
			{
				html_doc_stream_->pushInfo(htmlMarkup("h1", "Essential Matrix."));
				ostringstream os;
				os << std::endl
					<< "-------------------------------" << "<br>"
					<< "-- Robust Essential matrix: <" << I << "," << J << "> images: "
					<< fblib::utils::basename_part(camera_image_names_[I].image_name) << ","
					<< fblib::utils::basename_part(camera_image_names_[J].image_name) << "<br>"
					<< "-- Threshold: " << errorMax << "<br>"
					<< "-- Resection status: " << "OK" << "<br>"
					<< "-- Nb points used for robust Essential matrix estimation: "
					<< map_tracksCommon.size() << "<br>"
					<< "-- Nb points validated by robust estimation: "
					<< vec_inliers.size() << "<br>"
					<< "-- % points validated: "
					<< vec_inliers.size() / static_cast<float>(map_tracksCommon.size())
					<< "<br>"
					<< "-------------------------------" << "<br>";
				html_doc_stream_->pushInfo(os.str());

				html_doc_stream_->pushInfo(htmlMarkup("h2", "Residual of the robust estimation (Initial triangulation). Thresholded at: " + toString(errorMax)));

				JSXGraphWrapper jsxGraph;
				jsxGraph.init("InitialPairTriangulationResiduals", 600, 300);
				jsxGraph.addYChart(vec_triangulationResidual, "point");
				jsxGraph.addLine(0, errorMax, vec_triangulationResidual.size(), errorMax);
				jsxGraph.UnsuspendUpdate();
				std::pair< std::pair<double, double>, std::pair<double, double> > range = autoJSXGraphViewport<double>(vec_triangulationResidual);
				jsxGraph.setViewport(range);
				jsxGraph.close();
				html_doc_stream_->pushInfo(jsxGraph.toStr());


				html_doc_stream_->pushInfo(htmlMarkup("h2", "Histogram of residuals"));

				std::vector<double> xBin = histoResiduals.GetXbinsValue();
				range = autoJSXGraphViewport<double>(xBin, histoResiduals.GetHist());

				jsxGraph.init("InitialPairTriangulationKeptInfo", 600, 300);
				jsxGraph.addXYChart(xBin, histoResiduals.GetHist(), "line,point");
				jsxGraph.addLine(errorMax, 0, errorMax, histoResiduals.GetHist().front());
				jsxGraph.UnsuspendUpdate();
				jsxGraph.setViewport(range);
				jsxGraph.close();
				html_doc_stream_->pushInfo(jsxGraph.toStr());

				html_doc_stream_->pushInfo("<hr>");

				ofstream htmlFileStream(string(fblib::utils::folder_append_separator(out_dir_) +
					"Reconstruction_Report.html").c_str());
				htmlFileStream << html_doc_stream_->getDoc();
			}
			return !reconstructor_data_.map_3DPoints.empty();
		}

		/// Functor to sort a vector of pair given the pair's second value
		template<class T1, class T2, class Pred = std::less<T2> >
		struct sort_pair_second {
			bool operator()(const std::pair<T1, T2>&left,
				const std::pair<T1, T2>&right)
			{
				Pred p;
				return p(left.second, right.second);
			}
		};

		/// List the images that the greatest number of matches to the current 3D reconstruction.
		bool IncrementalReconstructionEngine::FindImagesWithPossibleResection(std::vector<size_t> & vec_possible_indexes)
		{
			vec_possible_indexes.clear();

			if (set_remaining_image_id_.empty() || map_reconstructed_.empty())  {
				return false;
			}

			// Estimate the image on which we could compute a resection safely
			// -> We first find the camera with the greatest number of matches
			//     with the current 3D existing 3D point => M
			// -> Then add any camera with at least 0.75M matches.
			// Keep only the best one.

			const double dPourcent = 0.75;

			std::vector< std::pair<size_t, size_t> > vec_putative; // ImageId, NbPutativeCommonPoint
			for (std::set<size_t>::const_iterator iter = set_remaining_image_id_.begin();
				iter != set_remaining_image_id_.end(); ++iter)
			{
				const size_t imageIndex = *iter;

				// Compute 2D - 3D possible content
				fblib::tracking::MapTracks map_tracksCommon;
				std::set<size_t> set_image_index;
				set_image_index.insert(imageIndex);
				TracksUtilsMap::GetTracksInImages(set_image_index, map_tracks_, map_tracksCommon);

				if (!map_tracksCommon.empty())
				{
					std::set<size_t> set_tracksIds;
					TracksUtilsMap::GetTracksIdVector(map_tracksCommon, &set_tracksIds);

					// Count the common possible putative point
					//  with the already 3D reconstructed trackId
					std::vector<size_t> vec_trackIdForResection;
					set_intersection(set_tracksIds.begin(), set_tracksIds.end(),
						reconstructor_data_.set_trackId.begin(),
						reconstructor_data_.set_trackId.end(),
						std::back_inserter(vec_trackIdForResection));

					vec_putative.push_back(make_pair(imageIndex, vec_trackIdForResection.size()));
				}
			}

			if (vec_putative.empty()) {
				return false;
			}
			else  {
				sort(vec_putative.begin(), vec_putative.end(), sort_pair_second<size_t, size_t, std::greater<size_t> >());

				size_t M = vec_putative[0].second;
				vec_possible_indexes.push_back(vec_putative[0].first);

				// TEMPORARY
				// FBLIB_INFO << std::endl << std::endl << "TEMPORARY return only the best image" << std::endl;
				// return true;
				// END TEMPORARY

				const size_t threshold = static_cast<size_t>(dPourcent * M);
				for (size_t i = 1; i < vec_putative.size() &&
					vec_putative[i].second > threshold; ++i)
				{
					vec_possible_indexes.push_back(vec_putative[i].first);
				}
				return true;
			}
		}

		/// Add to the current scene the desired image indexes.
		bool IncrementalReconstructionEngine::Resection(std::vector<size_t> & vec_possible_indexes)
		{
			bool is_ok = false;
			for (std::vector<size_t>::const_iterator iter = vec_possible_indexes.begin();
				iter != vec_possible_indexes.end();
				++iter)
			{
				vec_added_order_.push_back(*iter);
				bool bResect = Resection(*iter);
				is_ok |= bResect;
				if (!bResect) {
					// Resection was not possible (we remove the image from the remaining list)
					std::cerr << std::endl
						<< "Resection of image: " << *iter << " was not possible" << std::endl;
				}
				set_remaining_image_id_.erase(*iter);
			}
			return is_ok;
		}

		/// Add a single Image to the scene and triangulate new possible tracks
		bool IncrementalReconstructionEngine::Resection(size_t imageIndex)
		{
			FBLIB_INFO << std::endl
				<< "-------------------------------" << std::endl
				<< "-- Resection of camera index: " << imageIndex << std::endl
				<< "-------------------------------" << std::endl;

			// Compute 2D - 3D possible content
			fblib::tracking::MapTracks map_tracksCommon;
			std::set<size_t> set_image_index;
			set_image_index.insert(imageIndex);
			TracksUtilsMap::GetTracksInImages(set_image_index, map_tracks_, map_tracksCommon);

			std::set<size_t> set_tracksIds;
			TracksUtilsMap::GetTracksIdVector(map_tracksCommon, &set_tracksIds);

			// Intersect 3D reconstructed trackId with the one that contain the Image Id of interest
			std::set<size_t> set_trackIdForResection;
			set_intersection(set_tracksIds.begin(), set_tracksIds.end(),
				reconstructor_data_.set_trackId.begin(),
				reconstructor_data_.set_trackId.end(),
				std::inserter(set_trackIdForResection, set_trackIdForResection.begin()));

			// Load feature corresponding to imageIndex
			const std::vector<ScalePointFeature> & vec_featsImageIndex = map_features_[imageIndex];

			// Get back featId and tracksID that will be used for the resection
			std::vector<size_t> vec_featIdForResection;
			TracksUtilsMap::GetFeatIndexPerViewAndTrackId(map_tracksCommon,
				set_trackIdForResection,
				imageIndex,
				&vec_featIdForResection);

			FBLIB_INFO << std::endl << std::endl
				<< " Tracks in: " << imageIndex << std::endl
				<< " \t" << map_tracksCommon.size() << std::endl
				<< " #Reconstructed tracks:" << std::endl
				<< " \t" << reconstructor_data_.set_trackId.size() << std::endl
				<< " #Tracks Valid for resection:" << std::endl
				<< " \t" << set_trackIdForResection.size() << std::endl;

			// Normally it must not crash even if it have 0 matches
			if (set_trackIdForResection.empty())
			{
				// Too few matches (even 0 before....) images with empty connection
				set_remaining_image_id_.erase(imageIndex);
				return false;
			}

			// Create point_2d, and point_3d array
			Mat point_2d(2, set_trackIdForResection.size());
			Mat point_3d(3, set_trackIdForResection.size());

			size_t cpt = 0;
			std::set<size_t>::const_iterator iterTrackId = set_trackIdForResection.begin();
			for (std::vector<size_t>::const_iterator iterfeatId = vec_featIdForResection.begin();
				iterfeatId != vec_featIdForResection.end();
				++iterfeatId, ++iterTrackId, ++cpt)
			{
				point_3d.col(cpt) = reconstructor_data_.map_3DPoints[*iterTrackId];
				point_2d.col(cpt) = vec_featsImageIndex[*iterfeatId].coords().cast<double>();
			}

			//-------------
			std::vector<size_t> vec_inliers;
			Mat34 P;
			double errorMax = std::numeric_limits<double>::max();

			const fblib::feature::IntrinsicCameraInfo & intrinsicCam = vec_intrinsic_groups_[camera_image_names_[imageIndex].intrinsic_id];

			bool bResection = robustResection(
				std::make_pair(intrinsicCam.width, intrinsicCam.height),
				point_2d, point_3d,
				&vec_inliers,
				// If intrinsics guess exist use it, else use a standard 6 points pose resection
				(intrinsicCam.is_known_intrinsic == true) ? &intrinsicCam.camera_matrix : NULL,
				&P, &errorMax);

			FBLIB_INFO << std::endl
				<< "-------------------------------" << std::endl
				<< "-- Robust Resection of camera index: " << imageIndex << std::endl
				<< "-- Resection status: " << bResection << std::endl
				<< "-- #Points used for Resection: " << vec_featIdForResection.size() << std::endl
				<< "-- #Points validated by robust Resection: " << vec_inliers.size() << std::endl
				<< "-- Threshold: " << errorMax << std::endl
				<< "-------------------------------" << std::endl;

			if (is_html_report_)
			{
				ostringstream os;
				os << "Resection of Image index: <" << imageIndex << "> image: "
					<< fblib::utils::basename_part(camera_image_names_[imageIndex].image_name) << "<br> \n";
				html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

				os.str("");
				os << std::endl
					<< "-------------------------------" << "<br>"
					<< "-- Robust Resection of camera index: <" << imageIndex << "> image: "
					<< fblib::utils::basename_part(camera_image_names_[imageIndex].image_name) << "<br>"
					<< "-- Threshold: " << errorMax << "<br>"
					<< "-- Resection status: " << (bResection ? "OK" : "FAILED") << "<br>"
					<< "-- Nb points used for Resection: " << vec_featIdForResection.size() << "<br>"
					<< "-- Nb points validated by robust estimation: " << vec_inliers.size() << "<br>"
					<< "-- % points validated: "
					<< vec_inliers.size() / static_cast<float>(vec_featIdForResection.size()) << "<br>"
					<< "-------------------------------" << "<br>";
				html_doc_stream_->pushInfo(os.str());
			}

			if (!bResection) {
				return false;
			}

			//-- Add the camera to the _reconstruction data.
			reconstructor_data_.set_imagedId.insert(imageIndex);
			{
				Mat3 K, R;
				Vec3 t;
				KRt_From_P(P, &K, &R, &t);
				FBLIB_INFO << "\n Resection Calibration Matrix \n" << K << std::endl << std::endl;

				BrownPinholeCamera cam(K(0, 0), K(0, 2), K(1, 2), R, t);
				reconstructor_data_.map_Camera.insert(std::make_pair(imageIndex, cam));

				// Add the image to the reconstructed intrinsic group
				map_images_id_per_intrinsic_group_[map_intrinsic_id_per_image_id_[imageIndex]].push_back(imageIndex);

				// if intrinsic groups is empty fill a new:
				if (map_intrinsics_per_group_.find(map_intrinsic_id_per_image_id_[imageIndex]) == map_intrinsics_per_group_.end())
				{
					Vec6 & intrinsic = map_intrinsics_per_group_[map_intrinsic_id_per_image_id_[imageIndex]];
					intrinsic << cam._f, cam._ppx, cam._ppy, cam._k1, cam._k2, cam._k3;
				}
			}
			map_ac_threshold_.insert(std::make_pair(imageIndex, errorMax));
			set_remaining_image_id_.erase(imageIndex);

			// Evaluate residuals:
			std::vector<double> vec_ResectionResidual;
			for (size_t i = 0; i < point_3d.cols(); ++i)
			{
				double dResidual = PinholeCamera::Residual(P, point_3d.col(i), point_2d.col(i));
				vec_ResectionResidual.push_back(dResidual);
			}

			if (is_html_report_)
			{
				// Export graphical residual statistics
				html_doc_stream_->pushInfo(htmlMarkup("h2", "Residual of the robust estimation (Resection). Thresholded at: " + toString(errorMax)));
				JSXGraphWrapper jsxGraph;
				jsxGraph.init(std::string("ResectionResidual_residualPlot_" + toString(imageIndex)), 600, 300);
				jsxGraph.addYChart(vec_ResectionResidual, "point");
				jsxGraph.addLine(0, errorMax, vec_ResectionResidual.size(), errorMax);
				jsxGraph.UnsuspendUpdate();
				std::pair< std::pair<double, double>, std::pair<double, double> > range = autoJSXGraphViewport<double>(vec_ResectionResidual);
				jsxGraph.setViewport(range);
				jsxGraph.close();
				html_doc_stream_->pushInfo(jsxGraph.toStr());

				if (bResection)  {

					Histogram<double> histo(0, 2 * errorMax, 10);
					histo.Add(vec_ResectionResidual.begin(), vec_ResectionResidual.end());
					std::vector<double> xBin = histo.GetXbinsValue();
					std::pair< std::pair<double, double>, std::pair<double, double> > range = autoJSXGraphViewport<double>(xBin, histo.GetHist());

					jsxGraph.init(std::string("ResectionResidual_residualHisto_" + toString(imageIndex)), 600, 300);
					jsxGraph.addXYChart(xBin, histo.GetHist(), "line,point");
					jsxGraph.addLine(errorMax, 0, errorMax, histo.GetHist().front());
					jsxGraph.UnsuspendUpdate();
					jsxGraph.setViewport(range);
					jsxGraph.close();
					html_doc_stream_->pushInfo(jsxGraph.toStr());
				}
				html_doc_stream_->pushInfo("<hr>");
			}

			// Add new entry to reconstructed track and
			//  remove outlier from the tracks
			cpt = 0;
			std::vector<size_t>::iterator iterfeatId = vec_featIdForResection.begin();
			for (std::set<size_t>::const_iterator iterTrackId = set_trackIdForResection.begin();
				iterTrackId != set_trackIdForResection.end(); ++iterTrackId, ++cpt, ++iterfeatId)
			{
				if (vec_ResectionResidual[cpt] < errorMax) {
					// Inlier, add the point to the reconstructed track
					map_reconstructed_[*iterTrackId].insert(make_pair(imageIndex, *iterfeatId));
				}
				else {
					// Outlier remove this entry from the tracks
					map_tracks_[*iterTrackId].erase(imageIndex);
				}
			}

			// Add new possible tracks (triangulation)
			// Triangulate new possible tracks:
			// For all Union [ CurrentId, [PreviousReconstructedIds] ]
			//   -- If trackId not yet registered:
			//      -- Triangulate and add tracks id.

  {
	  // For all reconstructed image look if common content in the track
	  for (std::set<size_t>::const_iterator iterI = reconstructor_data_.set_imagedId.begin();
		  iterI != reconstructor_data_.set_imagedId.end(); ++iterI)
	  {
		  const size_t & indexI = *iterI;
		  if (indexI == imageIndex) { continue; }
		  size_t I = std::min(imageIndex, indexI);
		  size_t J = std::max(imageIndex, indexI);

		  // Compute possible content (match between indexI, indexJ)
		  map_tracksCommon.clear(); set_image_index.clear();
		  set_image_index.insert(I); set_image_index.insert(J);
		  TracksUtilsMap::GetTracksInImages(set_image_index, map_tracks_, map_tracksCommon);

		  if (map_tracksCommon.empty()) { continue; } // no common content

		  set_tracksIds.clear();
		  TracksUtilsMap::GetTracksIdVector(map_tracksCommon, &set_tracksIds);

		  //-- Compute if we have something to add to the scene ?
		  std::vector<size_t> vec_tracksToAdd;
		  //-- Do we have new Track to add ?
		  set_difference(set_tracksIds.begin(), set_tracksIds.end(),
			  reconstructor_data_.set_trackId.begin(), reconstructor_data_.set_trackId.end(),
			  back_inserter(vec_tracksToAdd));

		  if (!vec_tracksToAdd.empty())
		  {
			  const Mat34 & P1 = reconstructor_data_.map_Camera.find(I)->second.projection_matrix_;
			  const Mat34 & P2 = reconstructor_data_.map_Camera.find(J)->second.projection_matrix_;

			  const std::vector<ScalePointFeature> & vec_featI = map_features_[I];
			  const std::vector<ScalePointFeature> & vec_featJ = map_features_[J];

			  std::vector<IndexedMatch> vec_index;
			  TracksUtilsMap::TracksToIndexedMatches(map_tracksCommon, vec_tracksToAdd, &vec_index);

			  std::vector<Vec3> vec_3dPoint;
			  std::vector<double> vec_triangulationResidual;
			  vec_triangulationResidual.reserve(vec_index.size());
			  vec_3dPoint.reserve(vec_index.size());
			  triangulate2View_Vector(P1, P2,
				  vec_featI, vec_featJ,
				  vec_index, &vec_3dPoint, &vec_triangulationResidual);

			  bool bVisual = false; // Turn to true to see possible new triangulated points
			  if (bVisual)
			  {
				  ostringstream os;
				  os << "scene_" << I << "-" << J;
				  ExportToPly(vec_3dPoint,
					  fblib::utils::create_filespec(out_dir_, os.str(), "ply"));
			  }

			  // Analyze 3D reconstructed point
			  //  - Check positive depth
			  //  - Check angle (small angle leads imprecise triangulation)
			  const BrownPinholeCamera & cam1 = reconstructor_data_.map_Camera.find(I)->second;
			  const BrownPinholeCamera & cam2 = reconstructor_data_.map_Camera.find(J)->second;

			  // Threshold associated to each camera (min value of 4 pixels, to let BA refine if necessary)
			  const double maxThI = std::max(4.0, map_ac_threshold_[I]);
			  const double maxThJ = std::max(4.0, map_ac_threshold_[J]);

			  //- Add reconstructed point to the reconstruction data
			  size_t cardPointsBefore = reconstructor_data_.map_3DPoints.size();
			  for (size_t i = 0; i < vec_tracksToAdd.size(); ++i)
			  {
				  const size_t trackId = vec_tracksToAdd[i];
				  const Vec3 & cur3DPt = vec_3dPoint[i];

				  if (reconstructor_data_.set_trackId.find(trackId) == reconstructor_data_.set_trackId.end())
				  {
					  const Vec2 x1 = vec_featI[vec_index[i]._i].coords().cast<double>();
					  const Vec2 x2 = vec_featJ[vec_index[i]._j].coords().cast<double>();

					  bool bReproj =
						  cam1.Residual(cur3DPt, x1) < maxThI &&
						  cam2.Residual(cur3DPt, x2) < maxThJ;

					  if (bReproj
						  && cam1.Depth(cur3DPt) > 0
						  && cam2.Depth(cur3DPt) > 0)
					  {
						  double angle = BrownPinholeCamera::AngleBetweenRay(cam1, cam2, x1, x2);
						  if (angle > 2) {
							  reconstructor_data_.map_3DPoints[trackId] = vec_3dPoint[i];
							  reconstructor_data_.set_trackId.insert(trackId);
							  map_reconstructed_[trackId].insert(make_pair(I, vec_index[i]._i));
							  map_reconstructed_[trackId].insert(make_pair(J, vec_index[i]._j));
						  }
					  }
				  }
			  }

			  FBLIB_INFO << "--Triangulated 3D points [" << I << "-" << J << "]: "
				  << "\t #Validated/#Possible: " << reconstructor_data_.map_3DPoints.size() - cardPointsBefore
				  << "/" << vec_3dPoint.size() << std::endl
				  << " #3DPoint for the entire scene: " << reconstructor_data_.set_trackId.size() << std::endl;

			  if (bVisual) {
				  ostringstream file_name;
				  file_name << "incremental_" << indexI << "-" << imageIndex;
				  reconstructor_data_.exportToPlyFile(fblib::utils::create_filespec(out_dir_, file_name.str(), "ply"));
			  }
		  }
	  }
  }
			return true;
		}

		size_t IncrementalReconstructionEngine::badTrackRejector(double dPrecision)
		{
			// Go through the track and look for too large residual

			std::set<size_t> set_camIndex;
			std::transform(reconstructor_data_.map_Camera.begin(),
				reconstructor_data_.map_Camera.end(),
				std::inserter(set_camIndex, set_camIndex.begin()),
				RetrieveKey());

			std::map<size_t, std::set<size_t> > map_trackToErase; // trackid, imageIndexes
			std::set<size_t> set_trackToErase;

			for (std::map<size_t, Vec3>::const_iterator iter = reconstructor_data_.map_3DPoints.begin();
				iter != reconstructor_data_.map_3DPoints.end(); ++iter)
			{
				const size_t trackId = iter->first;
				const Vec3 & point_3d = iter->second;

				double maxAngle = 0.0;
				Vec3 originRay;
				// Look through the track and add point position
				const tracking::SubmapTrack & track = map_reconstructed_[trackId];
				for (tracking::SubmapTrack::const_iterator iterTrack = track.begin();
					iterTrack != track.end(); ++iterTrack)
				{
					const size_t imageId = iterTrack->first;
					const size_t featId = iterTrack->second;
					const BrownPinholeCamera & cam = reconstructor_data_.map_Camera.find(imageId)->second;

					if (set_camIndex.find(imageId) != set_camIndex.end())  {
						const std::vector<ScalePointFeature> & vec_feats = map_features_[imageId];
						const ScalePointFeature & ptFeat = vec_feats[featId];

						double dResidual2D = cam.Residual(point_3d, ptFeat.coords().cast<double>());

						const Vec3 camPos = cam.camera_center_;
						const Vec3 dir = (point_3d - camPos).normalized();
						if (iterTrack == track.begin())
						{
							originRay = dir;
						}
						else
						{
							double dot = originRay.dot(dir);
							double angle = R2D(acos(clamp(dot, -1.0 + 1.e-8, 1.0 - 1.e-8)));
							maxAngle = max(angle, maxAngle);
						}

						// If residual too large, remove the measurement
						if (dResidual2D > dPrecision) {
							map_trackToErase[trackId].insert(imageId);
						}
					}
				}
				if (maxAngle < 3)
				{
					for (tracking::SubmapTrack::const_iterator iterTrack = track.begin();
						iterTrack != track.end(); ++iterTrack)  {
						const size_t imageId = iterTrack->first;
						map_trackToErase[trackId].insert(imageId);
					}
				}
			}

			size_t rejectedTrack = 0, rejectedMeasurement = 0;

			for (std::map<size_t, std::set<size_t> >::const_iterator iterT = map_trackToErase.begin();
				iterT != map_trackToErase.end(); ++iterT)
			{
				const size_t trackId = iterT->first;

				const std::set<size_t> setI = iterT->second;
				// Erase the image index reference
				for (std::set<size_t>::const_iterator iterTT = setI.begin();
					iterTT != setI.end(); ++iterTT, ++rejectedMeasurement) {
					map_reconstructed_[trackId].erase(*iterTT);
				}

				// If remaining tracks is too small, remove it
				if (map_reconstructed_[trackId].size() < 2) {
					map_reconstructed_[trackId].clear();
					map_reconstructed_.erase(trackId);
					reconstructor_data_.set_trackId.erase(trackId);
					reconstructor_data_.map_3DPoints.erase(trackId);
					++rejectedTrack;
				}
			}

			FBLIB_INFO << "\n#rejected track: " << set_trackToErase.size() << std::endl
				<< "#rejected Entire track: " << rejectedTrack << std::endl
				<< "#rejected Measurement: " << rejectedMeasurement << std::endl;
			return rejectedTrack + rejectedMeasurement;
		}

		void IncrementalReconstructionEngine::ColorizeTracks(std::vector<Vec3> & vec_tracks_color) const
		{
			// Colorize each track
			//  Start with the most representative image
			//    and iterate to provide a color to each 3D point
			{
				ControlProgressDisplay my_progress_bar(map_reconstructed_.size(),
					std::cout,
					"\nCompute scene structure color\n");

				vec_tracks_color.resize(map_reconstructed_.size());

				//Build a list of contiguous index for the trackIds
				std::map<size_t, size_t> trackIds_to_contiguousIndexes;
				size_t cpt = 0;
				for (fblib::tracking::MapTracks::const_iterator it = map_reconstructed_.begin();
					it != map_reconstructed_.end(); ++it, ++cpt)
				{
					trackIds_to_contiguousIndexes[it->first] = cpt;
				}

				// The track list that will be colored (point removed during the process)
				fblib::tracking::MapTracks mapTrackToColor(map_reconstructed_);
				while (!mapTrackToColor.empty())
				{
					// Find the most representative image
					//  a. Count the number of visible point for each image
					//  b. Sort to find the most representative image

					std::map<size_t, size_t> map_IndexCardinal; // ImageIndex, Cardinal
					for (fblib::tracking::MapTracks::const_iterator
						iterT = mapTrackToColor.begin();
						iterT != mapTrackToColor.end();
					++iterT)
					{
						const size_t trackId = iterT->first;
						const tracking::SubmapTrack & track = mapTrackToColor[trackId];
						for (tracking::SubmapTrack::const_iterator iterTrack = track.begin();
							iterTrack != track.end(); ++iterTrack)
						{
							const size_t imageId = iterTrack->first;
							if (map_IndexCardinal.find(imageId) == map_IndexCardinal.end())
								map_IndexCardinal[imageId] = 1;
							else
								++map_IndexCardinal[imageId];
						}
					}

					// Find the image that is the most represented
					std::vector<size_t> vec_cardinal;
					std::transform(map_IndexCardinal.begin(),
						map_IndexCardinal.end(),
						std::back_inserter(vec_cardinal),
						RetrieveValue());
					std::vector< fblib::utils::SortIndexPacketDescend< size_t, size_t> > packet_vec(vec_cardinal.size());
					fblib::utils::SortIndexHelper(packet_vec, &vec_cardinal[0]);

					//First index is the image with the most of matches
					std::map<size_t, size_t>::const_iterator iterTT = map_IndexCardinal.begin();
					std::advance(iterTT, packet_vec[0].index);
					const size_t indexImage = iterTT->first;
					fblib::image::Image<fblib::image::RGBColor> image;
					ReadImage(
						fblib::utils::create_filespec(
						image_path_,
						fblib::utils::basename_part(camera_image_names_[indexImage].image_name),
						fblib::utils::extension_part(camera_image_names_[indexImage].image_name)).c_str(), &image);

					// Iterate through the track
					std::set<size_t> set_toRemove;
					for (fblib::tracking::MapTracks::const_iterator
						iterT = mapTrackToColor.begin();
						iterT != mapTrackToColor.end();
					++iterT)
					{
						const size_t trackId = iterT->first;
						const tracking::SubmapTrack & track = mapTrackToColor[trackId];
						tracking::SubmapTrack::const_iterator iterF = track.find(indexImage);

						if (iterF != track.end())
						{
							// Color the track
							const size_t featId = iterF->second;
							const ScalePointFeature & feat = map_features_.find(indexImage)->second[featId];
							fblib::image::RGBColor color = image(feat.y(), feat.x());

							vec_tracks_color[trackIds_to_contiguousIndexes[trackId]] = Vec3(color.r(), color.g(), color.b());
							set_toRemove.insert(trackId);
							++my_progress_bar;
						}
					}
					// Remove colored track
					for (std::set<size_t>::const_iterator iter = set_toRemove.begin();
						iter != set_toRemove.end(); ++iter)
					{
						mapTrackToColor.erase(*iter);
					}
				}
			}
		}

		void IncrementalReconstructionEngine::BundleAdjustment()
		{
			FBLIB_INFO << "--      BUNDLE ADJUSTMENT      --" << std::endl;

			//-- All the data that I must fill:
			using namespace std;

			const size_t nbCams = reconstructor_data_.map_Camera.size();
			const size_t nbIntrinsics = map_images_id_per_intrinsic_group_.size();
			const size_t nbPoints3D = reconstructor_data_.map_3DPoints.size();

			// Count the number of measurement (sum of the reconstructed track length)
			size_t nbmeasurements = 0;
			for (std::map<size_t, Vec3>::const_iterator iter = reconstructor_data_.map_3DPoints.begin();
				iter != reconstructor_data_.map_3DPoints.end();
				++iter)
			{
				const size_t trackId = iter->first;
				// Look through the track and add point position
				const tracking::SubmapTrack & track = map_reconstructed_[trackId];
				nbmeasurements += track.size();
			}

			FBLIB_INFO << "#Cams: " << nbCams << std::endl
				<< "#Intrinsics: " << nbIntrinsics << std::endl
				<< "#Points3D: " << nbPoints3D << std::endl
				<< "#measurements: " << nbmeasurements << std::endl;

			// Setup a BA problem
			using namespace fblib::sfm;
			BA_Problem_data_camMotionAndIntrinsic<6, 6> ba_problem;
			// Will refine extrinsics[R,t] per camera and grouped intrinsics [focal,ppx,ppy,k1,k2,k3].

			// Configure the size of the problem
			ba_problem.num_cameras_ = nbCams;
			ba_problem.num_intrinsic_ = nbIntrinsics;
			ba_problem.num_points_ = nbPoints3D;
			ba_problem.num_observations_ = nbmeasurements;

			ba_problem.point_index_.reserve(ba_problem.num_observations_);
			ba_problem.camera_index_extrinsic.reserve(ba_problem.num_observations_);
			ba_problem.camera_index_intrinsic.reserve(ba_problem.num_observations_);
			ba_problem.observations_.reserve(2 * ba_problem.num_observations_);

			ba_problem.num_parameters_ =
				6 * ba_problem.num_cameras_ // #[Rotation|translation]
				+ 6 * ba_problem.num_intrinsic_ // #[f,ppx,ppy,k1,k2,k3]
				+ 3 * ba_problem.num_points_; // #[X]
			ba_problem.parameters_.reserve(ba_problem.num_parameters_);

			// Setup extrinsic parameters
			std::set<size_t> set_camIndex;
			std::map<size_t, size_t> map_camIndexToNumber_extrinsic, map_camIndexToNumber_intrinsic;
			size_t cpt = 0;
			for (reconstructorHelper::Map_BrownPinholeCamera::const_iterator iter = reconstructor_data_.map_Camera.begin();
				iter != reconstructor_data_.map_Camera.end();  ++iter, ++cpt)
			{
				// in order to map camera index to contiguous number
				set_camIndex.insert(iter->first);
				map_camIndexToNumber_extrinsic.insert(std::make_pair(iter->first, cpt));

				Mat3 R = iter->second.rotation_matrix_;
				double angleAxis[3];
				ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
				// translation
				Vec3 t = iter->second.translation_vector_;
				ba_problem.parameters_.push_back(angleAxis[0]);
				ba_problem.parameters_.push_back(angleAxis[1]);
				ba_problem.parameters_.push_back(angleAxis[2]);
				ba_problem.parameters_.push_back(t[0]);
				ba_problem.parameters_.push_back(t[1]);
				ba_problem.parameters_.push_back(t[2]);
			}

			// Setup intrinsic parameters groups
			cpt = 0;
			for (std::map<size_t, Vec6 >::const_iterator iterIntrinsicGroup = map_intrinsics_per_group_.begin();
				iterIntrinsicGroup != map_intrinsics_per_group_.end();
				++iterIntrinsicGroup, ++cpt)
			{
				const Vec6 & intrinsic = iterIntrinsicGroup->second;
				ba_problem.parameters_.push_back(intrinsic(0)); // FOCAL_LENGTH
				ba_problem.parameters_.push_back(intrinsic(1)); // PRINCIPAL_POINT_X
				ba_problem.parameters_.push_back(intrinsic(2)); // PRINCIPAL_POINT_Y
				ba_problem.parameters_.push_back(intrinsic(3)); // K1
				ba_problem.parameters_.push_back(intrinsic(4)); // K2
				ba_problem.parameters_.push_back(intrinsic(5)); // K3
			}

			cpt = 0;
			// Link each camera to it's intrinsic group
			for (std::map<size_t, std::vector<size_t> >::const_iterator iterIntrinsicGroup = map_images_id_per_intrinsic_group_.begin();
				iterIntrinsicGroup != map_images_id_per_intrinsic_group_.end(); ++iterIntrinsicGroup, ++cpt)
			{
				const std::vector<size_t> vec_imagesId = iterIntrinsicGroup->second;
				for (std::vector<size_t>::const_iterator iter_vec = vec_imagesId.begin();
					iter_vec != vec_imagesId.end(); ++iter_vec)
				{
					const size_t camIndex = *iter_vec;
					map_camIndexToNumber_intrinsic.insert(std::make_pair(camIndex, cpt));
				}
			}

			// Setup 3D points
			for (std::map<size_t, Vec3>::const_iterator iter = reconstructor_data_.map_3DPoints.begin();
				iter != reconstructor_data_.map_3DPoints.end();
				++iter)
			{
				const Vec3 & point_3d = iter->second;
				ba_problem.parameters_.push_back(point_3d[0]);
				ba_problem.parameters_.push_back(point_3d[1]);
				ba_problem.parameters_.push_back(point_3d[2]);
			}

			// fill measurements
			cpt = 0;
			for (std::map<size_t, Vec3>::const_iterator iter = reconstructor_data_.map_3DPoints.begin();
				iter != reconstructor_data_.map_3DPoints.end();
				++iter)
			{
				const size_t trackId = iter->first;
				// Look through the track and add point position
				const tracking::SubmapTrack & track = map_reconstructed_[trackId];

				for (tracking::SubmapTrack::const_iterator iterTrack = track.begin();
					iterTrack != track.end();
					++iterTrack)
				{
					const size_t imageId = iterTrack->first;
					const size_t featId = iterTrack->second;

					// If imageId reconstructed:
					//  - Add measurements (the feature position)
					//  - Add camidx (map the image number to the camera index)
					//  - Add ptidx (the 3D corresponding point index) (must be increasing)

					if (set_camIndex.find(imageId) != set_camIndex.end())
					{
						const std::vector<ScalePointFeature> & vec_feats = map_features_[imageId];
						const ScalePointFeature & ptFeat = vec_feats[featId];

						ba_problem.observations_.push_back(ptFeat.x());
						ba_problem.observations_.push_back(ptFeat.y());

						ba_problem.point_index_.push_back(cpt);
						ba_problem.camera_index_extrinsic.push_back(map_camIndexToNumber_extrinsic[imageId]);
						ba_problem.camera_index_intrinsic.push_back(map_camIndexToNumber_intrinsic[imageId]);
					}
				}
				++cpt;
			}

			// Parameterization used to restrict camera intrinsics (Brown model or Pinhole Model).
			ceres::SubsetParameterization *constant_transform_parameterization = NULL;
			if (!is_refine_point_and_distortion_) {
				std::vector<int> vec_constant_PPAndRadialDisto;

				// Last five elements are ppx,ppy and radial disto factors.
				vec_constant_PPAndRadialDisto.push_back(1); // PRINCIPAL_POINT_X FIXED
				vec_constant_PPAndRadialDisto.push_back(2); // PRINCIPAL_POINT_Y FIXED
				vec_constant_PPAndRadialDisto.push_back(3); // K1 FIXED
				vec_constant_PPAndRadialDisto.push_back(4); // K2 FIXED
				vec_constant_PPAndRadialDisto.push_back(5); // K3 FIXED

				constant_transform_parameterization =
					new ceres::SubsetParameterization(6, vec_constant_PPAndRadialDisto);
			}

			// Create residuals for each observation in the bundle adjustment problem. The
			// parameters for cameras and points are added automatically.
			ceres::Problem problem;
			// Set a LossFunction to be less penalized by false measurements
			//  - set it to NULL if you don't want use a lossFunction.
			ceres::LossFunction * p_LossFunction = new ceres::HuberLoss(4.0);
			for (size_t i = 0; i < ba_problem.num_observations(); ++i) {
				// Each Residual block takes a point and a camera as input and outputs a 2
				// dimensional residual. Internally, the cost function stores the observed
				// image location and compares the reprojection against the observation.
				ceres::CostFunction* cost_function =
					new ceres::AutoDiffCostFunction<ErrorFunc_Refine_Camera_3DPoints, 2, 6, 6, 3>(
					new ErrorFunc_Refine_Camera_3DPoints(
					&ba_problem.observations()[2 * i + 0]));

				problem.AddResidualBlock(cost_function,
					p_LossFunction, // replaced by NULL if you don't want a LossFunction
					ba_problem.mutable_camera_intrisic_for_observation(i),
					ba_problem.mutable_camera_extrinsic_for_observation(i),
					ba_problem.mutable_point_for_observation(i));

				if (!is_refine_point_and_distortion_) {
					problem.SetParameterization(ba_problem.mutable_camera_intrisic_for_observation(i),
						constant_transform_parameterization);
				}
			}

			//-- Lock the first camera to better deal with scene orientation ambiguity
			if (vec_added_order_.size() > 0 && map_camIndexToNumber_extrinsic.size() > 0)
			{
				// First camera is the first one that have been used
				problem.SetParameterBlockConstant(
					ba_problem.mutable_camera_extrinsic_for_observation(
					map_camIndexToNumber_extrinsic[vec_added_order_[0]]));
			}

			// Configure a BA engine and run it
			//  Make Ceres automatically detect the bundle structure.
			ceres::Solver::Options options;
			options.preconditioner_type = ceres::JACOBI;
			options.linear_solver_type = ceres::SPARSE_SCHUR;
			if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
				options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			else
				if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
					options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
				else
				{
					// No sparse back end for Ceres.
					// Use dense solving
					options.linear_solver_type = ceres::DENSE_SCHUR;
				}
			options.minimizer_progress_to_stdout = false;
			options.logging_type = ceres::SILENT;
#ifdef USE_OPENMP
			options.num_threads = omp_get_max_threads();
			options.num_linear_solver_threads = omp_get_max_threads();
#endif // USE_OPENMP

			// Solve BA
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			FBLIB_INFO << summary.FullReport() << std::endl;

			// If no error, get back refined parameters
			if (summary.IsSolutionUsable())
			{
				// Display statistics about the minimization
				FBLIB_INFO << std::endl
					<< "Bundle Adjustment statistics:\n"
					<< " Initial RMSE: " << std::sqrt(summary.initial_cost / (ba_problem.num_observations_*2.)) << "\n"
					<< " Final RMSE: " << std::sqrt(summary.final_cost / (ba_problem.num_observations_*2.)) << "\n"
					<< std::endl;

				// Get back 3D points
				cpt = 0;
				for (std::map<size_t, Vec3>::iterator iter = reconstructor_data_.map_3DPoints.begin();
					iter != reconstructor_data_.map_3DPoints.end(); ++iter, ++cpt)
				{
					const double * pt = ba_problem.mutable_points() + cpt * 3;
					Vec3 & point_3d = iter->second;
					point_3d = Vec3(pt[0], pt[1], pt[2]);
				}

				// Get back camera external and intrinsic parameters
				for (reconstructorHelper::Map_BrownPinholeCamera::iterator iter = reconstructor_data_.map_Camera.begin();
					iter != reconstructor_data_.map_Camera.end(); ++iter)
				{
					const size_t imageId = iter->first;
					const size_t extrinsicId = map_camIndexToNumber_extrinsic[imageId];
					// Get back extrinsic pointer
					const double * camE = ba_problem.mutable_cameras_extrinsic() + extrinsicId * 6;
					Mat3 R;
					// angle axis to rotation matrix
					ceres::AngleAxisToRotationMatrix(camE, R.data());
					Vec3 t(camE[3], camE[4], camE[5]);

					// Get back the intrinsic group of the camera
					const size_t intrinsicId = map_camIndexToNumber_intrinsic[imageId];
					const double * camIntrinsics = ba_problem.mutable_cameras_intrinsic() + intrinsicId * 6;
					// Update the camera with update intrinsic and extrinsic parameters
					BrownPinholeCamera & sCam = iter->second;
					sCam = BrownPinholeCamera(
						camIntrinsics[OFFSET_FOCAL_LENGTH],
						camIntrinsics[OFFSET_PRINCIPAL_POINT_X],
						camIntrinsics[OFFSET_PRINCIPAL_POINT_Y],
						R,
						t,
						camIntrinsics[OFFSET_K1],
						camIntrinsics[OFFSET_K2],
						camIntrinsics[OFFSET_K3]);
				}

				//-- Update each intrinsic parameters group
				cpt = 0;
				for (std::map<size_t, Vec6 >::iterator iterIntrinsicGroup = map_intrinsics_per_group_.begin();
					iterIntrinsicGroup != map_intrinsics_per_group_.end();
					++iterIntrinsicGroup, ++cpt)
				{
					const double * camIntrinsics = ba_problem.mutable_cameras_intrinsic() + cpt * 6;
					Vec6 & intrinsic = iterIntrinsicGroup->second;
					intrinsic << camIntrinsics[OFFSET_FOCAL_LENGTH],
						camIntrinsics[OFFSET_PRINCIPAL_POINT_X],
						camIntrinsics[OFFSET_PRINCIPAL_POINT_Y],
						camIntrinsics[OFFSET_K1],
						camIntrinsics[OFFSET_K2],
						camIntrinsics[OFFSET_K3];

					FBLIB_INFO << " for camera Idx=[" << cpt << "]: " << std::endl
						<< "\t focal: " << camIntrinsics[OFFSET_FOCAL_LENGTH] << std::endl
						<< "\t ppx: " << camIntrinsics[OFFSET_PRINCIPAL_POINT_X] << std::endl
						<< "\t ppy: " << camIntrinsics[OFFSET_PRINCIPAL_POINT_Y] << std::endl
						<< "\t k1: " << camIntrinsics[OFFSET_K1] << std::endl
						<< "\t k2: " << camIntrinsics[OFFSET_K2] << std::endl
						<< "\t k3: " << camIntrinsics[OFFSET_K3] << std::endl;
				}
			}
		}

		double IncrementalReconstructionEngine::ComputeResidualsHistogram(Histogram<double> * histo)
		{
			std::set<size_t> set_camIndex;
			for (reconstructorHelper::Map_BrownPinholeCamera::const_iterator iter = reconstructor_data_.map_Camera.begin();
				iter != reconstructor_data_.map_Camera.end();
				++iter)
			{
				set_camIndex.insert(iter->first);
			}

			// For each 3D point sum their reprojection error

			std::vector<float> vec_residuals;
			vec_residuals.reserve(reconstructor_data_.map_3DPoints.size());

			for (std::map<size_t, Vec3>::const_iterator iter = reconstructor_data_.map_3DPoints.begin();
				iter != reconstructor_data_.map_3DPoints.end();
				++iter)
			{
				const size_t trackId = iter->first;
				const Vec3 & point_3d = iter->second;
				// Look through the track and add point position
				const tracking::SubmapTrack & track = map_reconstructed_[trackId];

				for (tracking::SubmapTrack::const_iterator iterTrack = track.begin();
					iterTrack != track.end();
					++iterTrack)
				{
					const size_t imageId = iterTrack->first;
					const size_t featId = iterTrack->second;

					if (set_camIndex.find(imageId) != set_camIndex.end())
					{
						const std::vector<ScalePointFeature> & vec_feats = map_features_[imageId];
						const ScalePointFeature & ptFeat = vec_feats[featId];
						const BrownPinholeCamera & cam = reconstructor_data_.map_Camera.find(imageId)->second;

						double dResidual = cam.Residual(point_3d, ptFeat.coords().cast<double>());
						vec_residuals.push_back(dResidual);
					}
				}
			}

			// Display statistics
			if (vec_residuals.size() > 1)
			{
				float min_residual, max_residual, mean_residual, median_residual;
				MinMaxMeanMedian<float>(vec_residuals.begin(), vec_residuals.end(),
					min_residual, max_residual, mean_residual, median_residual);
				if (histo)  {
					*histo = Histogram<double>(min_residual, max_residual, 10);
					histo->Add(vec_residuals.begin(), vec_residuals.end());
				}

				FBLIB_INFO << std::endl << std::endl;
				FBLIB_INFO << std::endl
					<< "IncrementalReconstructionEngine::ComputeResidualsMSE." << "\n"
					<< "\t-- #Tracks:\t" << map_reconstructed_.size() << std::endl
					<< "\t-- Residual min:\t" << min_residual << std::endl
					<< "\t-- Residual median:\t" << median_residual << std::endl
					<< "\t-- Residual max:\t " << max_residual << std::endl
					<< "\t-- Residual mean:\t " << mean_residual << std::endl;

				return mean_residual;
			}
			return -1.0;
		}
	}
} // namespace fblib
