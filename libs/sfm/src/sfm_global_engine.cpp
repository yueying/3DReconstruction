#include "sfm_precomp.h"
#include "mvg/feature/features.h"
#include "mvg/image/image.h"
#include "mvg/feature/indexed_match_utils.h"

#include "mvg/multiview/triangulation_nview.h"

#include "mvg/sfm/indexed_image_graph.h"
#include "mvg/sfm/indexed_image_graph_export.h"
#include "mvg/sfm/sfm_global_engine.h"
#include "mvg/feature/image_list_io_helper.h"
#include "mvg/sfm/sfm_robust.h"
#include "mvg/sfm/sfm_ply_helper.h"

#include "mvg/sfm/connected_component.h"

#include "mvg/sfm/sfm_global_tij_computation.h"
#include "mvg/utils/indexed_sort.h"
#include "mvg/utils/file_system.h"
#include "mvg/utils/svg_drawer.h"
#include "mvg/utils/stl_map.h"
#include "mvg/utils/histogram.h"

#include "mvg/sfm/linear_programming_interface.h"
#include "mvg/sfm/linear_programming_osi.h"
#ifdef MVG_HAVE_MOSEK
#include "mvg/sfm/linearProgrammingMOSEK.h"
#endif

#include "mvg/feature/estimator_acransac.h"
#include "mvg/feature/estimator_acransac_kernel_adaptator.h"

#undef DYNAMIC
#include "mvg/sfm/problem_data_container.h"
#include "mvg/sfm/pinhole_ceres_functor.h"
#include "mvg/sfm/sfm_bundle_adjustment_helper_tonly.h"

#include "mvg/sfm/sfm_global_engine_triplet_t_estimator.h"

#include "lemon/list_graph.h"
#include <lemon/connectivity.h>

// Rotation averaging
#include "mvg/multiview/rotation_averaging.h"
#include "mvg/utils/progress.h"
#include "mvg/utils/timer.h"

#include <numeric>
#include <iomanip>
#include <algorithm>
#include <functional>
#include <sstream>

using namespace mvg::utils;
using namespace mvg::sfm;
using namespace mvg::feature;

namespace mvg{
	namespace sfm{
		typedef mvg::feature::ScalePointFeature FeatureT;
		typedef std::vector<FeatureT> featsT;

		/// Return in radian the rotation amplitude of the given rotation matrix
		template <typename Mat>
		inline double
			getRotationMagnitude(const Mat & R)
		{
			double cos_theta = 0.5 * (R.trace() - 1.0);
			cos_theta = clamp(cos_theta, -1.0, 1.0);
			return std::acos(cos_theta);
		}

		GlobalReconstructionEngine::GlobalReconstructionEngine(const std::string & image_path,
			const std::string & matches_path, const std::string & out_dir, bool is_html_report)
			: ReconstructionEngine(image_path, matches_path, out_dir)
		{
			is_html_report_ = is_html_report;
			if (!mvg::utils::folder_exists(out_dir)) {
				mvg::utils::folder_create(out_dir);
			}
			if (is_html_report_)
			{
				html_doc_stream_ = shared_ptr<HtmlDocumentStream>(
					new HtmlDocumentStream("GlobalReconstructionEngine SFM report."));
				html_doc_stream_->pushInfo(
					htmlMarkup("h1", std::string("Current directory: ") +
					image_path));
				html_doc_stream_->pushInfo("<hr>");
			}
		}

		GlobalReconstructionEngine::~GlobalReconstructionEngine()
		{
			ofstream htmlFileStream(string(mvg::utils::folder_append_separator(out_dir_) +
				"Reconstruction_Report.html").c_str());
			htmlFileStream << html_doc_stream_->getDoc();
		}

		void GlobalReconstructionEngine::RotationInference(
			MapRelativeRT & map_relatives)
		{
			MVG_INFO
				<< "---------------\n"
				<< "-- INFERENCE on " << map_matches_fundamental_.size() << " EGs count.\n"
				<< "---------------" << std::endl
				<< " /!\\  /!\\  /!\\  /!\\  /!\\  /!\\  /!\\ \n"
				<< "--- ITERATED BAYESIAN INFERENCE IS NOT RELEASED, SEE C.ZACH CODE FOR MORE INFORMATION" << std::endl
				<< " /!\\  /!\\  /!\\  /!\\  /!\\  /!\\  /!\\ \n" << std::endl
				<< " A simple inference scheme is used here :" << std::endl
				<< "\t only the relative error composition to identity on cycle of length 3 is used." << std::endl;

			//-------------------
			// Triplet inference (test over the composition error)
			//-------------------
			std::vector< Triplet > vec_triplets;
			TripletListing(vec_triplets);
			//-- Rejection triplet that are 'not' identity rotation (error to identity > 2掳)
			TripletRotationRejection(vec_triplets, map_relatives);
		}

		bool GlobalReconstructionEngine::ComputeGlobalRotations(
			ERotationAveragingMethod eRotationAveragingMethod,
			const std::map<size_t, size_t> & map_camera_node_to_camera_index,
			const std::map<size_t, size_t> & map_camera_index_to_camera_node,
			const MapRelativeRT & map_relatives,
			std::map<size_t, Mat3> & map_globalR) const
		{
			// Build relative information for only the largest considered Connected Component
			// - it requires to change the camera indexes, because RotationAveraging is working only with
			//   index ranging in [0 - nbCam], (map_camera_node_to_camera_index utility)

			//- A. weight computation
			//- B. solve global rotation computation

			//- A. weight computation: for each pair w = min(1, (#PairMatches/Median(#PairsMatches)))
			std::vector<double> vec_relativeRotWeight;
			{
				vec_relativeRotWeight.reserve(map_relatives.size());
				{
					//-- Compute the median number of matches
					std::vector<double> vec_count;
					for (MapRelativeRT::const_iterator iter = map_relatives.begin();
						iter != map_relatives.end(); ++iter)
					{
						const mvg::sfm::RelativeInfo & rel = *iter;
						// Find the number of support point for this pair
						PairWiseMatches::const_iterator iterMatches = map_matches_fundamental_.find(rel.first);
						if (iterMatches != map_matches_fundamental_.end())
						{
							vec_count.push_back(iterMatches->second.size());
						}
					}
					float thTrustPair = (std::accumulate(vec_count.begin(), vec_count.end(), 0.0f) / vec_count.size()) / 2.0;

					for (MapRelativeRT::const_iterator iter = map_relatives.begin();
						iter != map_relatives.end(); ++iter)
					{
						const mvg::sfm::RelativeInfo & rel = *iter;
						float weight = 1.f; // the relative rotation correspondence point support
						PairWiseMatches::const_iterator iterMatches = map_matches_fundamental_.find(rel.first);
						if (iterMatches != map_matches_fundamental_.end())
						{
							weight = std::min((float)iterMatches->second.size() / thTrustPair, 1.f);
							vec_relativeRotWeight.push_back(weight);
						}
					}
				}
			}

			// Setup input data for global rotation computation
			std::vector<RelRotationData> vec_relativeRotEstimate;
			std::vector<double>::const_iterator iterW = vec_relativeRotWeight.begin();
			for (MapRelativeRT::const_iterator iter = map_relatives.begin();
				iter != map_relatives.end(); ++iter)
			{
				const mvg::sfm::RelativeInfo & rel = *iter;
				PairWiseMatches::const_iterator iterMatches = map_matches_fundamental_.find(rel.first);
				if (iterMatches != map_matches_fundamental_.end())
				{
					vec_relativeRotEstimate.push_back(RelRotationData(
						map_camera_node_to_camera_index.find(rel.first.first)->second,
						map_camera_node_to_camera_index.find(rel.first.second)->second,
						rel.second.first, *iterW));
					++iterW;
				}
			}

			//- B. solve global rotation computation
			bool bSuccess = false;
			std::vector<Mat3> vec_globalR(map_camera_index_to_camera_node.size());
			switch (eRotationAveragingMethod)
			{
			case ROTATION_AVERAGING_L2:
			{
				//- Solve the global rotation estimation problem:
				bSuccess = multiview::l2::L2RotationAveraging(map_camera_index_to_camera_node.size(),
					vec_relativeRotEstimate,
					vec_globalR);
			}
				break;
			case ROTATION_AVERAGING_L1:
			{
				using namespace mvg::multiview::l1;

				//- Solve the global rotation estimation problem:
				size_t nMainViewID = 0;
				std::vector<bool> vec_inliers;
				bSuccess = multiview::l1::GlobalRotationsRobust(
					vec_relativeRotEstimate, vec_globalR, nMainViewID, 0.0f, &vec_inliers);

				MVG_INFO << "\ninliers : " << std::endl;
				std::copy(vec_inliers.begin(), vec_inliers.end(), ostream_iterator<bool>(std::cout, " "));
				MVG_INFO << std::endl;
			}
				break;
			default:
				std::cerr << "Unknown rotation averaging method: " << (int)eRotationAveragingMethod << std::endl;
			}

			if (bSuccess)
			{
				//-- Setup the averaged rotation
				for (int i = 0; i < vec_globalR.size(); ++i)  {
					map_globalR[map_camera_index_to_camera_node.find(i)->second] = vec_globalR[i];
				}
			}
			else{
				std::cerr << "Global rotation solving failed." << std::endl;
			}

			return bSuccess;
		}

		bool GlobalReconstructionEngine::Process()
		{
			// 进行全局处理
			if (!readInputData())  {
				MVG_INFO << "\nError while parsing input data" << std::endl;
				return false;
			}

			//-- Export input graph
  {
	  typedef lemon::ListGraph Graph;
	  IndexedImageGraph putativeGraph(map_matches_fundamental_, vec_file_names_);

	  // Save the graph before cleaning:
	  exportToGraphvizData(
		  mvg::utils::create_filespec(out_dir_, "input_graph"),
		  putativeGraph.g);
  }


			mvg::utils::Timer total_reconstruction_timer;
			total_reconstruction_timer.Start();
			//-------------------
			// Only keep the largest biedge connected subgraph
			//-------------------

			if (!CleanGraph())
				return false;

			//-------------------
			// Compute relative R|t
			//-------------------

			MapRelativeRT map_relatives;
			{
				ComputeRelativeRt(map_relatives);
			}

			//-------------------
			// Rotation inference
			//-------------------

			//-- Putative triplets for relative translations computation
			std::vector< Triplet > vec_triplets;
			{
				mvg::utils::Timer timer_Inference;
				timer_Inference.Start();
				RotationInference(map_relatives);

				//-------------------
				// keep the largest biedge connected subgraph
				//-------------------
				if (!CleanGraph())
					return false;

				// recompute possible triplets since some nodes have been possibly removed
				TripletListing(vec_triplets);

				double time_Inference = timer_Inference.Stop();

				//---------------------------------------
				//-- Export geometric filtered matches
				//---------------------------------------
				std::ofstream file(string(matches_path_ + "/matches.filtered.txt").c_str());
				if (file.is_open())
					PairedIndexedMatchToStream(map_matches_fundamental_, file);
				file.close();

				//-------------------
				// List remaining camera node Id
				//-------------------
				std::set<size_t> set_indeximage;
				for (PairWiseMatches::const_iterator
					iter = map_matches_fundamental_.begin();
					iter != map_matches_fundamental_.end();
				++iter)
				{
					const size_t I = iter->first.first;
					const size_t J = iter->first.second;
					set_indeximage.insert(I);
					set_indeximage.insert(J);
				}
				// Build correspondences tables (cameraIds <=> GraphIds)
				for (std::set<size_t>::const_iterator iterSet = set_indeximage.begin();
					iterSet != set_indeximage.end(); ++iterSet)
				{
					map_camera_index_to_camera_node[std::distance(set_indeximage.begin(), iterSet)] = *iterSet;
					map_camera_node_to_camera_index[*iterSet] = std::distance(set_indeximage.begin(), iterSet);
				}

				MVG_INFO << "\n Remaining cameras after inference filter : \n"
					<< map_camera_index_to_camera_node.size() << " from a total of " << vec_file_names_.size() << std::endl;

				//-- Export statistics about the rotation inference step:
				if (is_html_report_)
				{
					std::ostringstream os;
					os << "Rotation inference.";
					html_doc_stream_->pushInfo("<hr>");
					html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

					os.str("");
					os << "-------------------------------" << "<br>"
						<< "-- #Camera count: " << set_indeximage.size() << " remains "
						<< "-- from " << vec_file_names_.size() << " input images.<br>"
						<< "-- timing : " << time_Inference << " second <br>"
						<< "-------------------------------" << "<br>";
					html_doc_stream_->pushInfo(os.str());
				}
			}

			//----------------------------
			// Rotation averaging
			//----------------------------

			std::map<std::size_t, Mat3> map_globalR;
			{
				MVG_INFO << "\n-------------------------------" << "\n"
					<< " Global rotations computation: " << "\n"
					<< "   - Ready to compute " << map_camera_index_to_camera_node.size() << " global rotations." << "\n"
					<< "     from " << map_relatives.size() << " relative rotations\n" << std::endl;

				int iChoice = 0;
				do
				{
					MVG_INFO
						<< "-------------------------------" << "\n"
						<< " Choose your rotation averaging method: " << "\n"
						<< "   - 1 -> MST based rotation + L1 rotation averaging" << "\n"
						<< "   - 2 -> dense L2 global rotation computation" << "\n";
				} while (!(std::cin >> iChoice) || iChoice < 0 || iChoice > ROTATION_AVERAGING_L2);

				if (!ComputeGlobalRotations(
					ERotationAveragingMethod(iChoice),
					map_camera_node_to_camera_index,
					map_camera_index_to_camera_node,
					map_relatives,
					map_globalR))
				{
					std::cerr << "Failed to compute the global rotations." << std::endl;
					return false;
				}
			}

			//-------------------
			// Relative translations estimation (Triplet based translation computation)
			//-------------------
			std::vector<mvg::sfm::RelativeInfo > vec_initialRijTijEstimates;
			PairWiseMatches newpairMatches;
			{
				MVG_INFO << "\n-------------------------------" << "\n"
					<< " Relative translations computation: " << "\n"
					<< "-------------------------------" << std::endl;

				// Compute putative translations with an edge coverage algorithm

				mvg::utils::Timer timerLP_triplet;
				timerLP_triplet.Start();
				ComputePutativeTranslationEdgesCoverage(map_globalR, vec_triplets, vec_initialRijTijEstimates, newpairMatches);
				double timeLP_triplet = timerLP_triplet.Stop();
				MVG_INFO << "TRIPLET COVERAGE TIMING : " << timeLP_triplet << " seconds" << std::endl;

				//-- Export triplet statistics:
				if (is_html_report_)
				{
					std::ostringstream os;
					os << "Triplet statistics.";
					html_doc_stream_->pushInfo("<hr>");
					html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

					os.str("");
					os << "-------------------------------" << "<br>"
						<< "-- #Effective triplet estimates: " << vec_initialRijTijEstimates.size() / 3
						<< " from " << vec_triplets.size() << " triplets.<br>"
						<< "-- resulting in " << vec_initialRijTijEstimates.size() << " translation estimation.<br>"
						<< "-- timing to obtain the relative translations : " << timeLP_triplet << " seconds.<br>"
						<< "-------------------------------" << "<br>";
					html_doc_stream_->pushInfo(os.str());
				}
			}

			//--Check the relative translation graph:
			//--> Consider only the connected component compound by the translation graph
			//-- Robust translation estimation can perform inference and remove some bad conditioned triplets

  {
	  std::set<size_t> set_representedImageIndex;
	  for (size_t i = 0; i < vec_initialRijTijEstimates.size(); ++i)
	  {
		  const mvg::sfm::RelativeInfo & rel = vec_initialRijTijEstimates[i];
		  set_representedImageIndex.insert(rel.first.first);
		  set_representedImageIndex.insert(rel.first.second);
	  }
	  MVG_INFO << "\n\n"
		  << "We targeting to estimates : " << map_globalR.size()
		  << " and we have estimation for : " << set_representedImageIndex.size() << " images" << std::endl;

	  //-- PRINT IMAGE THAT ARE NOT INSIDE THE TRIPLET GRAPH
	  for (std::map<size_t, Mat3>::const_iterator iter = map_globalR.begin();
		  iter != map_globalR.end(); ++iter)
	  {
		  if (set_representedImageIndex.find(iter->first) == set_representedImageIndex.end())
		  {
			  MVG_INFO << "Missing image index: " << iter->first << std::endl;
			  map_globalR.erase(map_camera_index_to_camera_node[iter->first]);
		  }
	  }

	  //- Build the map of camera index and node Ids that are listed by the triplets of translations
	  map_camera_index_to_camera_node.clear();
	  map_camera_node_to_camera_index.clear();
	  for (std::set<size_t>::const_iterator iterSet = set_representedImageIndex.begin();
		  iterSet != set_representedImageIndex.end(); ++iterSet)
	  {
		  map_camera_index_to_camera_node[std::distance(set_representedImageIndex.begin(), iterSet)] = *iterSet;
		  map_camera_node_to_camera_index[*iterSet] = std::distance(set_representedImageIndex.begin(), iterSet);
	  }

	  MVG_INFO << "\nRemaining cameras after inference filter : \n"
		  << map_camera_index_to_camera_node.size() << " from a total of " << vec_file_names_.size() << std::endl;
  }

			//-------------------
			//-- GLOBAL TRANSLATIONS ESTIMATION from initial triplets t_ij guess
			//-------------------

  {
	  const size_t iNview = map_camera_node_to_camera_index.size(); // The remaining camera nodes count in the graph

	  MVG_INFO << "\n-------------------------------" << "\n"
		  << " Global translations computation: " << "\n"
		  << "   - Ready to compute " << iNview << " global translations." << "\n"
		  << "     from " << vec_initialRijTijEstimates.size() << " relative translations\n" << std::endl;

	  //-- Update initial estimates in range [0->Ncam]
	  for (size_t i = 0; i < vec_initialRijTijEstimates.size(); ++i)
	  {
		  sfm::RelativeInfo & rel = vec_initialRijTijEstimates[i];
		  std::pair<size_t, size_t> newPair(
			  map_camera_node_to_camera_index[rel.first.first],
			  map_camera_node_to_camera_index[rel.first.second]);
		  rel.first = newPair;
	  }

	  mvg::utils::Timer timerLP_translation;
	  timerLP_translation.Start();
	  double gamma = -1.0;
	  std::vector<double> vec_solution;
	  {
		  vec_solution.resize(iNview * 3 + vec_initialRijTijEstimates.size() / 3 + 1);
		  using namespace mvg::sfm;
#ifdef MVG_HAVE_MOSEK
		  MOSEK_SolveWrapper solverLP(vec_solution.size());
#else
		  OSI_CLP_SolverWrapper solverLP(vec_solution.size());
#endif

		  sfm::Tifromtij_ConstraintBuilder_OneLambdaPerTrif constraint_builder(vec_initialRijTijEstimates);

		  LP_Constraints_Sparse constraint;
		  //-- Setup constraint and solver
		  constraint_builder.Build(constraint);
		  solverLP.setup(constraint);
		  //--
		  // Solving
		  bool bFeasible = solverLP.solve();
		  MVG_INFO << " \n Feasibility " << bFeasible << std::endl;
		  //--
		  if (bFeasible)
		  {
			  solverLP.getSolution(vec_solution);
			  gamma = vec_solution[vec_solution.size() - 1];
		  }
	  }

	  double timeLP_translation = timerLP_translation.Stop();

	  //-- Export triplet statistics:
	  if (is_html_report_)
	  {
		  std::ostringstream os;
		  os << "Translation fusion statistics.";
		  html_doc_stream_->pushInfo("<hr>");
		  html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

		  os.str("");
		  os << "-------------------------------" << "<br>"
			  << "-- #relative estimates: " << vec_initialRijTijEstimates.size()
			  << " converge with gamma: " << gamma << ".<br>"
			  << " timing (s): " << timeLP_translation << ".<br>"
			  << "-------------------------------" << "<br>";
		  html_doc_stream_->pushInfo(os.str());
	  }

	  MVG_INFO << "Found solution:\n";
	  std::copy(vec_solution.begin(), vec_solution.end(), std::ostream_iterator<double>(std::cout, " "));

	  std::vector<double> vec_camTranslation(iNview * 3, 0);
	  std::copy(&vec_solution[0], &vec_solution[iNview * 3], &vec_camTranslation[0]);

	  std::vector<double> vec_camRelLambdas(&vec_solution[iNview * 3], &vec_solution[iNview * 3 + vec_initialRijTijEstimates.size() / 3]);
	  MVG_INFO << "\ncam position: " << std::endl;
	  std::copy(vec_camTranslation.begin(), vec_camTranslation.end(), std::ostream_iterator<double>(std::cout, " "));
	  MVG_INFO << "\ncam Lambdas: " << std::endl;
	  std::copy(vec_camRelLambdas.begin(), vec_camRelLambdas.end(), std::ostream_iterator<double>(std::cout, " "));

	  // Build a Pinhole camera for each considered Id
	  std::vector<Vec3>  vec_C;
	  for (size_t i = 0; i < iNview; ++i)
	  {
		  Vec3 t(vec_camTranslation[i * 3], vec_camTranslation[i * 3 + 1], vec_camTranslation[i * 3 + 2]);
		  const size_t camNodeId = map_camera_index_to_camera_node[i];
		  const Mat3 & Ri = map_globalR[camNodeId];
		  const Mat3 & camera_matrix_ = vec_intrinsic_groups_[0].camera_matrix;   // The same K matrix is used by all the camera
		  map_camera_[camNodeId] = PinholeCamera(camera_matrix_, Ri, t);
		  //-- Export camera center
		  vec_C.push_back(map_camera_[camNodeId].camera_center_);
	  }
	  exportToPly(vec_C, mvg::utils::create_filespec(out_dir_, "cameraPath", "ply"));
  }

			//-------------------
			//-- Initial triangulation of the scene from the computed global motions
			//-------------------
  {
	  // Build tracks from selected triplets (Union of all the validated triplet tracks (newpairMatches))
	  //  triangulate tracks
	  //  refine translations
	  {
		  TracksBuilder tracks_builder;
		  tracks_builder.Build(newpairMatches);
		  tracks_builder.Filter(3);
		  tracks_builder.ExportToSTL(map_selected_tracks_);

		  MVG_INFO << std::endl << "Track stats" << std::endl;
		  {
			  std::ostringstream osTrack;
			  //-- Display stats :
			  //    - number of images
			  //    - number of tracks
			  std::set<size_t> set_imagesId;
			  TracksUtilsMap::ImageIdInTracks(map_selected_tracks_, set_imagesId);
			  osTrack << "------------------" << "\n"
				  << "-- Tracks Stats --" << "\n"
				  << " Tracks number: " << tracks_builder.NbTracks() << "\n"
				  << " Images Id: " << "\n";
			  std::copy(set_imagesId.begin(),
				  set_imagesId.end(),
				  std::ostream_iterator<size_t>(osTrack, ", "));
			  osTrack << "\n------------------" << "\n";

			  std::map<size_t, size_t> map_Occurence_TrackLength;
			  TracksUtilsMap::TracksLength(map_selected_tracks_, map_Occurence_TrackLength);
			  osTrack << "TrackLength, Occurrence" << "\n";
			  for (std::map<size_t, size_t>::const_iterator iter = map_Occurence_TrackLength.begin();
				  iter != map_Occurence_TrackLength.end(); ++iter)  {
				  osTrack << "\t" << iter->first << "\t" << iter->second << "\n";
			  }
			  osTrack << "\n";
			  MVG_INFO << osTrack.str();
		  }
	  }

	  // Triangulation of all the tracks
	  vec_all_scenes_.resize(map_selected_tracks_.size());
	  {
		  std::vector<double> vec_residuals;
		  vec_residuals.resize(map_selected_tracks_.size());
		  std::set<size_t> set_idx_to_remove;

		  ControlProgressDisplay my_progress_bar_triangulation(map_selected_tracks_.size(),
			  std::cout,
			  "Initial triangulation:\n");

#ifdef USE_OPENMP
#pragma omp parallel for //schedule(dynamic, 1)
#endif
		  for (int idx = 0; idx < map_selected_tracks_.size(); ++idx)
		  {
			  MapTracks::const_iterator iterTracks = map_selected_tracks_.begin();
			  std::advance(iterTracks, idx);

			  const SubmapTrack & subTrack = iterTracks->second;

			  // Look to the features required for the triangulation task
			  size_t index = 0;
			  Triangulation trianObj;
			  for (SubmapTrack::const_iterator iter = subTrack.begin(); iter != subTrack.end(); ++iter, ++index) {

				  const size_t imaIndex = iter->first;
				  const size_t featIndex = iter->second;
				  const ScalePointFeature & pt = map_features_[imaIndex][featIndex];
				  // Build the P matrix
				  trianObj.add(map_camera_[imaIndex].projection_matrix_, pt.coords().cast<double>());
			  }

			  // Compute the 3D point and keep point index with negative depth
			  const Vec3 Xs = trianObj.compute();
			  vec_all_scenes_[idx] = Xs;
			  if (trianObj.minDepth() < 0)  {
				  set_idx_to_remove.insert(idx);
			  }

			  //-- Compute residual over all the projections
			  for (SubmapTrack::const_iterator iter = subTrack.begin(); iter != subTrack.end(); ++iter) {
				  const size_t imaIndex = iter->first;
				  const size_t featIndex = iter->second;
				  const ScalePointFeature & pt = map_features_[imaIndex][featIndex];
				  vec_residuals[idx] = map_camera_[imaIndex].Residual(Xs, pt.coords().cast<double>());
			  }

#ifdef USE_OPENMP
#pragma omp critical
#endif
		{
			++my_progress_bar_triangulation;
		}
		  }

		  //-- Remove useless tracks and 3D points
	  {
		  std::vector<Vec3> vec_allScenes_cleaned;
		  for (size_t i = 0; i < vec_all_scenes_.size(); ++i)
		  {
			  if (find(set_idx_to_remove.begin(), set_idx_to_remove.end(), i) == set_idx_to_remove.end())
			  {
				  vec_allScenes_cleaned.push_back(vec_all_scenes_[i]);
			  }
		  }
		  vec_all_scenes_ = vec_allScenes_cleaned;

		  for (std::set<size_t>::const_iterator iter = set_idx_to_remove.begin();
			  iter != set_idx_to_remove.end(); ++iter)
		  {
			  map_selected_tracks_.erase(*iter);
		  }
		  MVG_INFO << "\n Tracks have been removed : " << set_idx_to_remove.size() << std::endl;
	  }
		  exportToPly(vec_all_scenes_, mvg::utils::create_filespec(out_dir_, "raw_pointCloud_LP", "ply"));

		  {
			  // Display some statistics of reprojection errors
			  MVG_INFO << "\n\nResidual statistics:\n" << std::endl;
			  MinMaxMeanMedian<double>(vec_residuals.begin(), vec_residuals.end());
			  double min, max, mean, median;
			  MinMaxMeanMedian<double>(vec_residuals.begin(), vec_residuals.end(), min, max, mean, median);

			  Histogram<float> histo(0.f, *max_element(vec_residuals.begin(), vec_residuals.end())*1.1f);
			  histo.Add(vec_residuals.begin(), vec_residuals.end());
			  MVG_INFO << std::endl << "Residual Error pixels : " << std::endl << histo.ToString() << std::endl;

			  // Histogram between 0 and 10 pixels
			  {
				  MVG_INFO << "\n Histogram between 0 and 10 pixels: \n";
				  Histogram<float> histo(0.f, 10.f, 20);
				  histo.Add(vec_residuals.begin(), vec_residuals.end());
				  MVG_INFO << std::endl << "Residual Error pixels : " << std::endl << histo.ToString() << std::endl;
			  }

			  //-- Export initial triangulation statistics
			  if (is_html_report_)
			  {
				  std::ostringstream os;
				  os << "Initial triangulation statistics.";
				  html_doc_stream_->pushInfo("<hr>");
				  html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

				  os.str("");
				  os << "-------------------------------" << "<br>"
					  << "-- #tracks: " << map_selected_tracks_.size() << ".<br>"
					  << "-- #observation: " << vec_residuals.size() << ".<br>"
					  << "-- residual mean (RMSE): " << std::sqrt(mean) << ".<br>"
					  << "-------------------------------" << "<br>";
				  html_doc_stream_->pushInfo(os.str());
			  }
		  }
  }
		}

			//-------------------
			//-- Bundle Adjustment on translation and structure
			//-------------------

			bundleAdjustment_t_Xi(map_camera_, vec_all_scenes_, map_selected_tracks_);
			bundleAdjustment_Rt_Xi(map_camera_, vec_all_scenes_, map_selected_tracks_);

			//-- Export statistics about the global process
			if (is_html_report_)
			{
				std::ostringstream os;
				os << "Global calibration summary triangulation statistics.";
				html_doc_stream_->pushInfo("<hr>");
				html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

				os.str("");
				os << "-------------------------------" << "<br>"
					<< "-- Have calibrated: " << map_camera_.size() << " from "
					<< vec_file_names_.size() << " input images.<br>"
					<< "-- The scene contains " << map_selected_tracks_.size() << " 3D points.<br>"
					<< "-- Total reconstruction time (Inference, global rot, translation fusion, Ba1, Ba2): "
					<< total_reconstruction_timer.Stop() << " seconds.<br>"
					<< "-------------------------------" << "<br>";
				html_doc_stream_->pushInfo(os.str());
			}

			MVG_INFO << std::endl
				<< "-------------------------------" << "\n"
				<< "-- Have calibrated: " << map_camera_.size() << " from "
				<< vec_file_names_.size() << " input images.\n"
				<< "-- The scene contains " << map_selected_tracks_.size() << " 3D points.\n"
				<< "-- Total reconstruction time (Inference, global rot, translation fusion, Ba1, Ba2): "
				<< total_reconstruction_timer.Stop() << " seconds.\n"
				<< "Relative rotations time was excluded\n"
				<< "-------------------------------" << std::endl;

			//-- Export the scene (cameras and structures) to a container
			{
				// Cameras
				for (std::map<size_t, PinholeCamera >::const_iterator iter = map_camera_.begin();
					iter != map_camera_.end();  ++iter)
				{
					const PinholeCamera & cam = iter->second;
					reconstructor_data_.map_Camera[iter->first] = BrownPinholeCamera(cam.projection_matrix_);

					// reconstructed camera
					reconstructor_data_.set_imagedId.insert(iter->first);
				}

				// Structure
				size_t i = 0;
				for (std::vector<Vec3>::const_iterator iter = vec_all_scenes_.begin();
					iter != vec_all_scenes_.end();
					++iter, ++i)
				{
					const Vec3 & point_3d = *iter;
					reconstructor_data_.map_3d_points[i] = point_3d;
					reconstructor_data_.set_trackId.insert(i);
				}
			}

			return true;
	}

		bool testIntrinsicsEquality(
			feature::IntrinsicCameraInfo const &ci1,
			feature::IntrinsicCameraInfo const &ci2)
		{
			return ci1.camera_matrix == ci2.camera_matrix;
		}

		bool GlobalReconstructionEngine::readInputData()
		{
			if (!mvg::utils::is_folder(image_path_) ||
				!mvg::utils::is_folder(matches_path_) ||
				!mvg::utils::is_folder(out_dir_))
			{
				std::cerr << std::endl
					<< "One of the required directory is not a valid directory" << std::endl;
				return false;
			}

			// a. Read images names
			std::string lists_file = mvg::utils::create_filespec(matches_path_, "lists", "txt");
			std::string sComputedMatchesFile_E = mvg::utils::create_filespec(matches_path_, "matches.e", "txt");
			if (!mvg::utils::is_file(lists_file) ||
				!mvg::utils::is_file(sComputedMatchesFile_E))
			{
				std::cerr << std::endl
					<< "One of the input required file is not a present (lists.txt, matches.e.txt)" << std::endl;
				return false;
			}

			// a. Read images names
  {
	  if (!mvg::feature::loadImageList(
		  lists_file, vec_camera_image_names_,
		  vec_intrinsic_groups_
		  ))
	  {
		  std::cerr << "\nEmpty image list." << std::endl;
		  return false;
	  }
	  else
	  {
		  // Check there is only one intrinsic group
		  std::vector<mvg::feature::IntrinsicCameraInfo>::iterator iterF =
			  std::unique(vec_intrinsic_groups_.begin(), vec_intrinsic_groups_.end(), testIntrinsicsEquality);
		  vec_intrinsic_groups_.resize(std::distance(vec_intrinsic_groups_.begin(), iterF));
		  if (vec_intrinsic_groups_.size() == 1)
		  {
			  for (size_t i = 0; i < vec_camera_image_names_.size(); ++i)
				  vec_camera_image_names_[i].intrinsic_id = 0;
		  }
		  else
		  {
			  MVG_INFO << "There is more than one focal group in the lists.txt file." << std::endl
				  << "Only one focal group is supported for the global calibration chain" << std::endl;
			  return false;
		  }

		  for (size_t i = 0; i < vec_camera_image_names_.size(); ++i)
		  {
			  vec_file_names_.push_back(vec_camera_image_names_[i].image_name);
		  }
	  }
  }

			// b. Read matches (Essential)
			if (!feature::pairedIndexedMatchImport(sComputedMatchesFile_E, map_matches_fundamental_)) {
				std::cerr << "Unable to read the Essential matrix matches" << std::endl;
				return false;
			}

			// Read features:
			for (size_t i = 0; i < vec_file_names_.size(); ++i)  {
				const size_t camIndex = i;
				if (!LoadFeatsFromFile(
					mvg::utils::create_filespec(matches_path_, mvg::utils::basename_part(vec_file_names_[camIndex]), ".feat"),
					map_features_[camIndex])) {
					std::cerr << "Bad reading of feature files" << std::endl;
					return false;
				}
			}
			return true;
		}

		bool GlobalReconstructionEngine::CleanGraph()
		{
			// Create a graph from pairwise correspondences:
			// - remove not biedge connected component,
			// - keep the largest connected component.

			typedef lemon::ListGraph Graph;
			IndexedImageGraph putativeGraph(map_matches_fundamental_, vec_file_names_);

			// Save the graph before cleaning:
			exportToGraphvizData(
				mvg::utils::create_filespec(out_dir_, "initialGraph"),
				putativeGraph.g);

			// Remove not bi-edge connected edges
			typedef Graph::EdgeMap<bool> EdgeMapAlias;
			EdgeMapAlias cutMap(putativeGraph.g);

			if (lemon::biEdgeConnectedCutEdges(putativeGraph.g, cutMap) > 0)
			{
				// Some edges must be removed because they don't follow the biEdge condition.
				typedef Graph::EdgeIt EdgeIterator;
				EdgeIterator itEdge(putativeGraph.g);
				for (EdgeMapAlias::MapIt it(cutMap); it != INVALID; ++it, ++itEdge)
				{
					if (*it)
						putativeGraph.g.erase(itEdge); // remove the not bi-edge element
				}
			}

			// Graph is bi-edge connected, but still many connected components can exist
			// Keep only the largest one
			PairWiseMatches matches_filtered;
			int connected_component_count = lemon::countConnectedComponents(putativeGraph.g);
			MVG_INFO << "\n"
				<< "GlobalReconstructionEngine::CleanGraph() :: => connected Component : "
				<< connected_component_count << std::endl;
			if (connected_component_count > 1)
			{
				// Keep only the largest connected component
				// - list all CC size
				// - if the largest one is meet, keep all the edges that belong to this node

				const std::map<size_t, std::set<lemon::ListGraph::Node> > map_subgraphs = ExportGraphToMapSubgraphs(putativeGraph.g);
				size_t count = std::numeric_limits<size_t>::min();
				std::map<size_t, std::set<lemon::ListGraph::Node> >::const_iterator iterLargestCC = map_subgraphs.end();
				for (std::map<size_t, std::set<lemon::ListGraph::Node> >::const_iterator iter = map_subgraphs.begin();
					iter != map_subgraphs.end(); ++iter)
				{
					if (iter->second.size() > count)  {
						count = iter->second.size();
						iterLargestCC = iter;
					}
					MVG_INFO << "Connected component of size : " << iter->second.size() << std::endl;
				}

				//-- Remove all nodes that are not listed in the largest CC
				for (std::map<size_t, std::set<lemon::ListGraph::Node> >::const_iterator iter = map_subgraphs.begin();
					iter != map_subgraphs.end(); ++iter)
				{
					if (iter == iterLargestCC)
					{
						// list all nodes and outgoing edges and update the matching list
						const std::set<lemon::ListGraph::Node> & ccSet = iter->second;
						for (std::set<lemon::ListGraph::Node>::const_iterator iter2 = ccSet.begin();
							iter2 != ccSet.end(); ++iter2)
						{
							typedef Graph::OutArcIt OutArcIt;
							for (OutArcIt e(putativeGraph.g, *iter2); e != INVALID; ++e)
							{
								size_t Idu = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.target(e)];
								size_t Idv = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.source(e)];
								PairWiseMatches::iterator iterF = map_matches_fundamental_.find(std::make_pair(Idu, Idv));
								if (iterF != map_matches_fundamental_.end())
								{
									matches_filtered.insert(*iterF);
								}
								iterF = map_matches_fundamental_.find(std::make_pair(Idv, Idu));
								if (iterF != map_matches_fundamental_.end())
								{
									matches_filtered.insert(*iterF);
								}
							}
						}
						// update the matching list
						map_matches_fundamental_ = matches_filtered;
					}
					else
					{
						// remove the edges from the graph
						const std::set<lemon::ListGraph::Node> & ccSet = iter->second;
						for (std::set<lemon::ListGraph::Node>::const_iterator iter2 = ccSet.begin();
							iter2 != ccSet.end(); ++iter2)
						{
							typedef Graph::OutArcIt OutArcIt;
							for (OutArcIt e(putativeGraph.g, *iter2); e != INVALID; ++e)
							{
								putativeGraph.g.erase(e);
							}
							//putativeGraph.g.erase(*iter2);
						}
					}
				}
			}

			// Save the graph after cleaning:
			exportToGraphvizData(
				mvg::utils::create_filespec(out_dir_, "cleanedGraph"),
				putativeGraph.g);

			MVG_INFO << "\n"
				<< "Cardinal of nodes: " << lemon::countNodes(putativeGraph.g) << "\n"
				<< "Cardinal of edges: " << lemon::countEdges(putativeGraph.g) << std::endl
				<< std::endl;

			return true;
		}

		void GlobalReconstructionEngine::ComputeRelativeRt(
			MapRelativeRT & vec_relatives)
		{
			// For each pair, compute the rotation from pairwise point matches:

			ControlProgressDisplay my_progress_bar(map_matches_fundamental_.size(), std::cout, "\n", " ", "ComputeRelativeRt\n");
#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
			for (int i = 0; i < map_matches_fundamental_.size(); ++i)
			{
				PairWiseMatches::const_iterator iter = map_matches_fundamental_.begin();
				std::advance(iter, i);

				const size_t I = iter->first.first;
				const size_t J = iter->first.second;

				const std::vector<IndexedMatch> & vec_matchesInd = iter->second;

				Mat x1(2, vec_matchesInd.size()), x2(2, vec_matchesInd.size());
				for (size_t k = 0; k < vec_matchesInd.size(); ++k)
				{
					x1.col(k) = map_features_[I][vec_matchesInd[k]._i].coords().cast<double>();
					x2.col(k) = map_features_[J][vec_matchesInd[k]._j].coords().cast<double>();
				}

				Mat3 E;
				std::vector<size_t> vec_inliers;

				std::pair<size_t, size_t> imageSize_I(
					vec_intrinsic_groups_[vec_camera_image_names_[I].intrinsic_id].width,
					vec_intrinsic_groups_[vec_camera_image_names_[I].intrinsic_id].height);

				std::pair<size_t, size_t> imageSize_J(
					vec_intrinsic_groups_[vec_camera_image_names_[J].intrinsic_id].width,
					vec_intrinsic_groups_[vec_camera_image_names_[J].intrinsic_id].height);

				const Mat3 K = vec_intrinsic_groups_[vec_camera_image_names_[I].intrinsic_id].camera_matrix;

				double errorMax = std::numeric_limits<double>::max();
				double maxExpectedError = 2.5;
				if (!robustEssential(K, K,
					x1, x2,
					&E, &vec_inliers,
					imageSize_I, imageSize_J,
					&errorMax,
					maxExpectedError))
				{
					std::cerr << " /!\\ Robust estimation failed to compute E for this pair"
						<< std::endl;
					continue;
				}
				else
				{
					//--> Estimate the best possible Rotation/Translation from E
					Mat3 R;
					Vec3 t;
					if (!estimateRtFromE(K, K, x1, x2, E, vec_inliers, &R, &t))
					{
						MVG_INFO << " /!\\ Failed to compute initial R|t for the initial pair"
							<< std::endl;
						continue;
					}
					else
					{
						PinholeCamera cam1(K, Mat3::Identity(), Vec3::Zero());
						PinholeCamera cam2(K, R, t);

						std::vector<Vec3> vec_all_scenes;
						vec_all_scenes.resize(x1.cols());
						for (size_t k = 0; k < x1.cols(); ++k) {
							const Vec2 & x1_ = x1.col(k),
								&x2_ = x2.col(k);
							Vec3 X;
							TriangulateDLT(cam1.projection_matrix_, x1_, cam2.projection_matrix_, x2_, &X);
							vec_all_scenes[k] = X;
						}

						// Refine Xis, tis and Ris (Bundle Adjustment)
		{
			using namespace std;

			const size_t nbCams = 2;
			const size_t nbPoints3D = vec_all_scenes.size();

			// Count the number of measurement (sum of the reconstructed track length)
			size_t nbmeasurements = x1.cols() * 2;

			// 创建一个BA问题
			BAProblemData<6> ba_problem; // Will refine translation and 3D points

			// Configure the size of the problem
			ba_problem.num_cameras_ = nbCams;
			ba_problem.num_points_ = nbPoints3D;
			ba_problem.num_observations_ = nbmeasurements;

			ba_problem.point_index_.reserve(ba_problem.num_observations_);
			ba_problem.camera_index_.reserve(ba_problem.num_observations_);
			ba_problem.observations_.reserve(2 * ba_problem.num_observations_);

			ba_problem.num_parameters_ = 6 * ba_problem.num_cameras_ + 3 * ba_problem.num_points_;
			ba_problem.parameters_.reserve(ba_problem.num_parameters_);

			// fill camera
			{
				Mat3 R = cam1.rotation_matrix_;
				double angleAxis[3];
				ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);

				// translation
				Vec3 t = cam1.translation_vector_;

				ba_problem.parameters_.push_back(angleAxis[0]);
				ba_problem.parameters_.push_back(angleAxis[1]);
				ba_problem.parameters_.push_back(angleAxis[2]);
				ba_problem.parameters_.push_back(t[0]);
				ba_problem.parameters_.push_back(t[1]);
				ba_problem.parameters_.push_back(t[2]);
			}
		  {
			  Mat3 R = cam2.rotation_matrix_;
			  double angleAxis[3];
			  ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);

			  // translation
			  Vec3 t = cam2.translation_vector_;

			  ba_problem.parameters_.push_back(angleAxis[0]);
			  ba_problem.parameters_.push_back(angleAxis[1]);
			  ba_problem.parameters_.push_back(angleAxis[2]);
			  ba_problem.parameters_.push_back(t[0]);
			  ba_problem.parameters_.push_back(t[1]);
			  ba_problem.parameters_.push_back(t[2]);
		  }

			// fill 3D points
			for (std::vector<Vec3>::const_iterator iter = vec_all_scenes.begin();
				iter != vec_all_scenes.end();
				++iter)
			{
				const Vec3 & point_3d = *iter;
				ba_problem.parameters_.push_back(point_3d[0]);
				ba_problem.parameters_.push_back(point_3d[1]);
				ba_problem.parameters_.push_back(point_3d[2]);
			}

			// fill the measurements
			for (size_t k = 0; k < x1.cols(); ++k) {
				const Vec2 & x1_ = x1.col(k), &x2_ = x2.col(k);

				const Mat3 & K = cam1.camera_matrix_;

				double ppx = K(0, 2), ppy = K(1, 2);
				ba_problem.observations_.push_back(x1_(0) - ppx);
				ba_problem.observations_.push_back(x1_(1) - ppy);
				ba_problem.point_index_.push_back(k);
				ba_problem.camera_index_.push_back(0);

				ba_problem.observations_.push_back(x2_(0) - ppx);
				ba_problem.observations_.push_back(x2_(1) - ppy);
				ba_problem.point_index_.push_back(k);
				ba_problem.camera_index_.push_back(1);
			}

			// Create residuals for each observation in the bundle adjustment problem. The
			// parameters for cameras and points are added automatically.
			ceres::Problem problem;
			// Set a LossFunction to be less penalized by false measurements
			//  - set it to NULL if you don't want use a lossFunction.
			ceres::LossFunction * p_LossFunction = new ceres::HuberLoss(Square(2.0));
			for (size_t i = 0; i < ba_problem.num_observations(); ++i) {
				// Each Residual block takes a point and a camera as input and outputs a 2
				// dimensional residual. Internally, the cost function stores the observed
				// image location and compares the reprojection against the observation.

				ceres::CostFunction* cost_function =
					new ceres::AutoDiffCostFunction<PinholeReprojectionError_Rt, 2, 6, 3>(
					new PinholeReprojectionError_Rt(
					&ba_problem.observations()[2 * i + 0], K(0, 0)));

				problem.AddResidualBlock(cost_function,
					p_LossFunction,
					ba_problem.mutable_camera_for_observation(i),
					ba_problem.mutable_point_for_observation(i));
			}

			// Configure a BA engine and run it
			//  Make Ceres automatically detect the bundle structure.
			ceres::Solver::Options options;
			// Use a dense back-end since we only consider a two view problem
			options.linear_solver_type = ceres::DENSE_SCHUR;
			options.minimizer_progress_to_stdout = false;
			options.logging_type = ceres::SILENT;

			// Solve BA
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			// If no error, get back refined parameters
			if (summary.IsSolutionUsable())
			{
				// Get back 3D points
				size_t k = 0;
				for (std::vector<Vec3>::iterator iter = vec_all_scenes.begin();
					iter != vec_all_scenes.end(); ++iter, ++k)
				{
					const double * pt = ba_problem.mutable_points() + k * 3;
					Vec3 & point_3d = *iter;
					point_3d = Vec3(pt[0], pt[1], pt[2]);
				}

				// Get back camera 1
			{
				const double * cam = ba_problem.mutable_cameras() + 0 * 6;
				Mat3 R;
				// angle axis to rotation matrix
				ceres::AngleAxisToRotationMatrix(cam, R.data());

				Vec3 t(cam[3], cam[4], cam[5]);

				// Update the camera
				Mat3 K = cam1.camera_matrix_;
				PinholeCamera & sCam = cam1;
				sCam = PinholeCamera(K, R, t);
			}
				// Get back camera 2
			{
				const double * cam = ba_problem.mutable_cameras() + 1 * 6;
				Mat3 R;
				// angle axis to rotation matrix
				ceres::AngleAxisToRotationMatrix(cam, R.data());

				Vec3 t(cam[3], cam[4], cam[5]);

				// Update the camera
				Mat3 K = cam2.camera_matrix_;
				PinholeCamera & sCam = cam2;
				sCam = PinholeCamera(K, R, t);
			}
				RelativeCameraMotion(cam1.rotation_matrix_, cam1.translation_vector_, cam2.rotation_matrix_, cam2.translation_vector_, &R, &t);
			}
		}

#ifdef USE_OPENMP
#pragma omp critical
#endif
		{
			vec_relatives[iter->first] = std::make_pair(R, t);
			++my_progress_bar;
		}

					}
				}
			}
				}

		void GlobalReconstructionEngine::TripletListing(std::vector< Triplet > & vec_triplets) const
		{
			vec_triplets.clear();

			IndexedImageGraph putativeGraph(map_matches_fundamental_, vec_file_names_);

			ListTriplets<IndexedImageGraph::GraphT>(putativeGraph.g, vec_triplets);

			//Change triplets to ImageIds
			for (size_t i = 0; i < vec_triplets.size(); ++i)
			{
				Triplet & triplet = vec_triplets[i];
				size_t I = triplet.i, J = triplet.j, K = triplet.k;
				I = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.nodeFromId(I)];
				J = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.nodeFromId(J)];
				K = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.nodeFromId(K)];
				size_t triplet_[3] = { I, J, K };
				std::sort(&triplet_[0], &triplet_[3]);
				triplet = Triplet(triplet_[0], triplet_[1], triplet_[2]);
			}
		}

		void GlobalReconstructionEngine::TripletRotationRejection(
			std::vector< Triplet > & vec_triplets,
			MapRelativeRT & map_relatives)
		{
			MapRelativeRT map_relatives_validated;

			// DETECTION OF ROTATION OUTLIERS
			std::vector< Triplet > vec_triplets_validated;

			std::vector<float> vec_errToIdentityPerTriplet;
			vec_errToIdentityPerTriplet.reserve(vec_triplets.size());
			// Compute for each length 3 cycles: the composition error
			//  Error to identity rotation.
			for (size_t i = 0; i < vec_triplets.size(); ++i)
			{
				const Triplet & triplet = vec_triplets[i];
				size_t I = triplet.i, J = triplet.j, K = triplet.k;

				//-- Find the three rotations
				const std::pair<size_t, size_t> ij = std::make_pair(I, J);
				const std::pair<size_t, size_t> ji = std::make_pair(J, I);

				Mat3 RIJ;
				if (map_relatives.find(ij) != map_relatives.end())
					RIJ = map_relatives.find(ij)->second.first;
				else
					RIJ = map_relatives.find(ji)->second.first.transpose();

				const std::pair<size_t, size_t> jk = std::make_pair(J, K);
				const std::pair<size_t, size_t> kj = std::make_pair(K, J);

				Mat3 RJK;
				if (map_relatives.find(jk) != map_relatives.end())
					RJK = map_relatives.find(jk)->second.first;
				else
					RJK = map_relatives.find(kj)->second.first.transpose();

				const std::pair<size_t, size_t> ki = std::make_pair(K, I);
				const std::pair<size_t, size_t> ik = std::make_pair(I, K);

				Mat3 RKI;
				if (map_relatives.find(ki) != map_relatives.end())
					RKI = map_relatives.find(ki)->second.first;
				else
					RKI = map_relatives.find(ik)->second.first.transpose();

				Mat3 Rot_To_Identity = RIJ * RJK * RKI; // motion composition
				float angularErrorDegree = static_cast<float>(R2D(getRotationMagnitude(Rot_To_Identity)));
				vec_errToIdentityPerTriplet.push_back(angularErrorDegree);

				if (angularErrorDegree < 2.0f)
				{
					vec_triplets_validated.push_back(triplet);

					if (map_relatives.find(ij) != map_relatives.end())
						map_relatives_validated[ij] = map_relatives.find(ij)->second;
					else
						map_relatives_validated[ji] = map_relatives.find(ji)->second;

					if (map_relatives.find(jk) != map_relatives.end())
						map_relatives_validated[jk] = map_relatives.find(jk)->second;
					else
						map_relatives_validated[kj] = map_relatives.find(kj)->second;

					if (map_relatives.find(ki) != map_relatives.end())
						map_relatives_validated[ki] = map_relatives.find(ki)->second;
					else
						map_relatives_validated[ik] = map_relatives.find(ik)->second;
				}
			}

			map_relatives = map_relatives_validated;

			// Display statistics about rotation triplets error:
			MVG_INFO << "\nStatistics about rotation triplets:" << std::endl;
			MinMaxMeanMedian<float>(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end());

			std::sort(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end());

			Histogram<float> histo(0.0f, *max_element(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end()), 180);
			histo.Add(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end());

			SvgHistogram histosvg;
			histosvg.draw(histo.GetHist(),
				std::pair<float, float>(0.0f, *max_element(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end())),
				mvg::utils::create_filespec(this->out_dir_, "Triplet_Rotation_Residual_180.svg"),
				600, 300);

			histo = Histogram<float>(0.0f, *max_element(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end()), 20);
			histo.Add(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end());

			histosvg.draw(histo.GetHist(),
				std::pair<float, float>(0.0f, *max_element(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end())),
				mvg::utils::create_filespec(this->out_dir_, "Triplet_Rotation_Residual_20.svg"),
				600, 300);

			typedef lemon::ListGraph Graph;
			IndexedImageGraph putativeGraph(map_matches_fundamental_, vec_file_names_);

			Graph::EdgeMap<bool> edge_filter(putativeGraph.g, false);
			Graph::NodeMap<bool> node_filter(putativeGraph.g, true);

			typedef SubGraph<Graph > subGraphT;
			subGraphT sg(putativeGraph.g, node_filter, edge_filter);

			// Look all edges of the graph and look if exist in one triplet
			for (Graph::EdgeIt iter(putativeGraph.g); iter != INVALID; ++iter)
			{
				size_t Idu = (*putativeGraph.map_nodeMapIndex)[sg.u(iter)];
				size_t Idv = (*putativeGraph.map_nodeMapIndex)[sg.v(iter)];
				//-- Look if the edge Idu,Idv exists in the trifocal tensor list
				for (size_t i = 0; i < vec_triplets_validated.size(); ++i)
				{
					const Triplet & triplet = vec_triplets_validated[i];
					if (triplet.contain(std::make_pair(Idu, Idv)))
					{
						edge_filter[iter] = true;
						break;
					}
				}
			}

			exportToGraphvizData(
				mvg::utils::create_filespec(out_dir_, "cleanedGraphTripletRotation"),
				sg);

			{
				MVG_INFO << "\nTriplets filtering based on error on cycles \n";
				MVG_INFO << "Before : " << vec_triplets.size() << " triplets \n"
					<< "After : " << vec_triplets_validated.size() << std::endl;
				MVG_INFO << "There is " << lemon::countConnectedComponents(sg)
					<< " Connected Component in the filtered graph" << std::endl;
			}

			vec_triplets.clear();
			vec_triplets = vec_triplets_validated;

			size_t removedEdgesCount = 0;

			//-- Remove false edges from the rejected triplets
			{
				for (Graph::EdgeIt iter(putativeGraph.g); iter != INVALID; ++iter)
				{
					if (!edge_filter[iter])
					{
						removedEdgesCount++;

						size_t Idu = (*putativeGraph.map_nodeMapIndex)[sg.u(iter)];
						size_t Idv = (*putativeGraph.map_nodeMapIndex)[sg.v(iter)];

						//-- Clean relatives matches
						PairWiseMatches::iterator iterF = map_matches_fundamental_.find(std::make_pair(Idu, Idv));
						if (iterF != map_matches_fundamental_.end())
						{
							map_matches_fundamental_.erase(iterF);
						}
						else
						{
							iterF = map_matches_fundamental_.find(std::make_pair(Idv, Idu));
							if (iterF != map_matches_fundamental_.end())
								map_matches_fundamental_.erase(iterF);
						}

						//-- Clean relative motions
						MapRelativeRT::iterator iterF2 = map_relatives.find(std::make_pair(Idu, Idv));
						if (iterF2 != map_relatives.end())
						{
							map_relatives.erase(iterF2);
						}
						else
						{
							iterF2 = map_relatives.find(std::make_pair(Idv, Idu));
							if (iterF2 != map_relatives.end())
								map_relatives.erase(iterF2);
						}
					}
				}
			}

			MVG_INFO << "\n Relatives edges removed by triplet checking : " << removedEdgesCount << std::endl;
		}

		void GlobalReconstructionEngine::bundleAdjustment_t_Xi(
			std::map<size_t, PinholeCamera> & map_camera,
			std::vector<Vec3> & vec_all_scenes,
			const MapTracks & map_tracks_selected)
		{
			using namespace std;

			const size_t nbCams = map_camera.size();
			const size_t nbPoints3D = vec_all_scenes.size();

			// Count the number of measurement (sum of the reconstructed track length)
			size_t nbmeasurements = 0;
			for (MapTracks::const_iterator iterTracks = map_tracks_selected.begin();
				iterTracks != map_tracks_selected.end(); ++iterTracks)
			{
				const SubmapTrack & subTrack = iterTracks->second;
				nbmeasurements += subTrack.size();
			}

			// Setup a BA problem
			BAProblemData<3> ba_problem; // Will refine translation and 3D points

			// Configure the size of the problem
			ba_problem.num_cameras_ = nbCams;
			ba_problem.num_points_ = nbPoints3D;
			ba_problem.num_observations_ = nbmeasurements;

			ba_problem.point_index_.reserve(ba_problem.num_observations_);
			ba_problem.camera_index_.reserve(ba_problem.num_observations_);
			ba_problem.observations_.reserve(2 * ba_problem.num_observations_);

			ba_problem.num_parameters_ = 3 * ba_problem.num_cameras_ + 3 * ba_problem.num_points_;
			ba_problem.parameters_.reserve(ba_problem.num_parameters_);

			// fill camera
			std::vector<double> vec_Rot(map_camera.size() * 3, 0.0);
			size_t i = 0;
			std::map<size_t, size_t> map_camIndexToNumber;
			for (MapCamera::const_iterator iter = map_camera.begin();
				iter != map_camera.end();  ++iter, ++i)
			{
				// in order to map camera index to contiguous number
				map_camIndexToNumber.insert(std::make_pair(iter->first, i));

				Mat3 R = iter->second.rotation_matrix_;
				double angleAxis[3];
				ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
				vec_Rot[i * 3] = angleAxis[0];
				vec_Rot[i * 3 + 1] = angleAxis[1];
				vec_Rot[i * 3 + 2] = angleAxis[2];

				// translation
				const Vec3 & t = iter->second.translation_vector_;
				ba_problem.parameters_.push_back(t[0]);
				ba_problem.parameters_.push_back(t[1]);
				ba_problem.parameters_.push_back(t[2]);
			}

			// fill 3D points
			for (std::vector<Vec3>::const_iterator iter = vec_all_scenes.begin();
				iter != vec_all_scenes.end();
				++iter)
			{
				const Vec3 & point_3d = *iter;
				ba_problem.parameters_.push_back(point_3d[0]);
				ba_problem.parameters_.push_back(point_3d[1]);
				ba_problem.parameters_.push_back(point_3d[2]);
			}

			// fill the measurements
			i = 0;
			for (MapTracks::const_iterator iterTracks = map_tracks_selected.begin();
				iterTracks != map_tracks_selected.end(); ++iterTracks, ++i)
			{
				// Look through the track and add point position
				const tracking::SubmapTrack & track = iterTracks->second;

				for (tracking::SubmapTrack::const_iterator iterTrack = track.begin();
					iterTrack != track.end();
					++iterTrack)
				{
					size_t imageId = iterTrack->first;
					size_t featId = iterTrack->second;

					// If imageId reconstructed :
					//  - Add measurements (the feature position)
					//  - Add camidx (map the image number to the camera index)
					//  - Add ptidx (the 3D corresponding point index) (must be increasing)

					//if ( set_camIndex.find(imageId) != set_camIndex.end())
					{
						const std::vector<ScalePointFeature> & vec_feats = map_features_[imageId];
						const ScalePointFeature & ptFeat = vec_feats[featId];
						const PinholeCamera & cam = map_camera[imageId];

						double ppx = cam.camera_matrix_(0, 2), ppy = cam.camera_matrix_(1, 2);
						ba_problem.observations_.push_back(ptFeat.x() - ppx);
						ba_problem.observations_.push_back(ptFeat.y() - ppy);

						ba_problem.point_index_.push_back(i);
						ba_problem.camera_index_.push_back(map_camIndexToNumber[imageId]);
					}
				}
			}

			// The same K matrix is used by all the camera
			const Mat3 camera_matrix_ = vec_intrinsic_groups_[0].camera_matrix;

			// Create residuals for each observation in the bundle adjustment problem. The
			// parameters for cameras and points are added automatically.
			ceres::Problem problem;
			// Set a LossFunction to be less penalized by false measurements
			//  - set it to NULL if you don't want use a lossFunction.
			ceres::LossFunction * p_LossFunction = new ceres::HuberLoss(Square(4.0));
			for (i = 0; i < ba_problem.num_observations(); ++i) {
				// Each Residual block takes a point and a camera as input and outputs a 2
				// dimensional residual. Internally, the cost function stores the observed
				// image location and compares the reprojection against the observation.

				ceres::CostFunction* cost_function =
					new ceres::AutoDiffCostFunction<PinholeReprojectionError_t, 2, 3, 3>(
					new PinholeReprojectionError_t(
					&ba_problem.observations()[2 * i + 0],
					camera_matrix_(0, 0),
					&vec_Rot[ba_problem.camera_index_[i] * 3]));

				problem.AddResidualBlock(cost_function,
					p_LossFunction,
					ba_problem.mutable_camera_for_observation(i),
					ba_problem.mutable_point_for_observation(i));
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
					// No sparse backend for Ceres.
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
			MVG_INFO << summary.FullReport() << std::endl;

			// If no error, get back refined parameters
			if (summary.IsSolutionUsable())
			{
				// Get back 3D points
				i = 0;
				for (std::vector<Vec3>::iterator iter = vec_all_scenes.begin();
					iter != vec_all_scenes.end(); ++iter, ++i)
				{
					const double * pt = ba_problem.mutable_points() + i * 3;
					Vec3 & point_3d = *iter;
					point_3d = Vec3(pt[0], pt[1], pt[2]);
				}
				exportToPly(vec_all_scenes, mvg::utils::create_filespec(out_dir_, "raw_pointCloud_BA_T_Xi", "ply"));

				// Get back camera
				i = 0;
				for (MapCamera::iterator iter = map_camera.begin();
					iter != map_camera.end(); ++iter, ++i)
				{
					const double * cam = ba_problem.mutable_cameras() + i * 3;

					Vec3 t(cam[0], cam[1], cam[2]);
					PinholeCamera & sCam = iter->second;
					Mat3 K = sCam.camera_matrix_;
					Mat3 R = sCam.rotation_matrix_;
					sCam = PinholeCamera(K, R, t);
				}

	{
		//-- Export First bundle adjustment statistics
		if (is_html_report_)
		{
			std::ostringstream os;
			os << "First Bundle Adjustment (Ti, Xjs) statistics.";
			html_doc_stream_->pushInfo("<hr>");
			html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

			os.str("");
			os << "-------------------------------" << "<br>"
				<< "-- #tracks: " << map_tracks_selected.size() << ".<br>"
				<< "-- #observation: " << ba_problem.num_observations_ << ".<br>"
				<< "-- initial residual mean (RMSE): " << std::sqrt(summary.initial_cost / ba_problem.num_observations_) << ".<br>"
				<< "-- residual mean (RMSE): " << std::sqrt(summary.final_cost / ba_problem.num_observations_) << ".<br>"
				<< "-- Nb Steps required until convergence : " << summary.num_successful_steps + summary.num_unsuccessful_steps << ".<br>"
				<< "-------------------------------" << "<br>";
			html_doc_stream_->pushInfo(os.str());
		}
	}
			}
		}

		void GlobalReconstructionEngine::bundleAdjustment_Rt_Xi(
			MapCamera & map_camera,
			std::vector<Vec3> & vec_all_scenes,
			const MapTracks & map_tracks_selected)
		{
			using namespace std;

			const size_t nbCams = map_camera.size();
			const size_t nbPoints3D = vec_all_scenes.size();

			// Count the number of measurement (sum of the reconstructed track length)
			size_t nbmeasurements = 0;
			for (MapTracks::const_iterator iterTracks = map_tracks_selected.begin();
				iterTracks != map_tracks_selected.end(); ++iterTracks)
			{
				const SubmapTrack & subTrack = iterTracks->second;
				nbmeasurements += subTrack.size();
			}

			// Setup a BA problem
			BAProblemData<6> ba_problem; // Will refine translation and 3D points

			// Configure the size of the problem
			ba_problem.num_cameras_ = nbCams;
			ba_problem.num_points_ = nbPoints3D;
			ba_problem.num_observations_ = nbmeasurements;

			ba_problem.point_index_.reserve(ba_problem.num_observations_);
			ba_problem.camera_index_.reserve(ba_problem.num_observations_);
			ba_problem.observations_.reserve(2 * ba_problem.num_observations_);

			ba_problem.num_parameters_ = 6 * ba_problem.num_cameras_ + 3 * ba_problem.num_points_;
			ba_problem.parameters_.reserve(ba_problem.num_parameters_);

			// fill camera
			size_t i = 0;
			std::map<size_t, size_t> map_camIndexToNumber;
			for (MapCamera::const_iterator iter = map_camera.begin();
				iter != map_camera.end();  ++iter, ++i)
			{
				// in order to map camera index to contiguous number
				map_camIndexToNumber.insert(std::make_pair(iter->first, i));

				const Mat3 & R = iter->second.rotation_matrix_;
				double angleAxis[3];
				ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);

				// translation
				const Vec3 & t = iter->second.translation_vector_;

				ba_problem.parameters_.push_back(angleAxis[0]);
				ba_problem.parameters_.push_back(angleAxis[1]);
				ba_problem.parameters_.push_back(angleAxis[2]);
				ba_problem.parameters_.push_back(t[0]);
				ba_problem.parameters_.push_back(t[1]);
				ba_problem.parameters_.push_back(t[2]);
			}

			// fill 3D points
			for (std::vector<Vec3>::const_iterator iter = vec_all_scenes.begin();
				iter != vec_all_scenes.end();
				++iter)
			{
				const Vec3 & point_3d = *iter;
				ba_problem.parameters_.push_back(point_3d[0]);
				ba_problem.parameters_.push_back(point_3d[1]);
				ba_problem.parameters_.push_back(point_3d[2]);
			}

			// fill the measurements
			i = 0;
			for (MapTracks::const_iterator iterTracks = map_tracks_selected.begin();
				iterTracks != map_tracks_selected.end(); ++iterTracks, ++i)
			{
				// Look through the track and add point position
				const tracking::SubmapTrack & track = iterTracks->second;

				for (tracking::SubmapTrack::const_iterator iterTrack = track.begin();
					iterTrack != track.end();
					++iterTrack)
				{
					const size_t imageId = iterTrack->first;
					const size_t featId = iterTrack->second;

					// If imageId reconstructed :
					//  - Add measurements (the feature position)
					//  - Add camidx (map the image number to the camera index)
					//  - Add ptidx (the 3D corresponding point index) (must be increasing)

					//if ( set_camIndex.find(imageId) != set_camIndex.end())
					{
						const std::vector<ScalePointFeature> & vec_feats = map_features_[imageId];
						const ScalePointFeature & ptFeat = vec_feats[featId];
						const PinholeCamera & cam = map_camera[imageId];

						double ppx = cam.camera_matrix_(0, 2), ppy = cam.camera_matrix_(1, 2);
						ba_problem.observations_.push_back(ptFeat.x() - ppx);
						ba_problem.observations_.push_back(ptFeat.y() - ppy);

						ba_problem.point_index_.push_back(i);
						ba_problem.camera_index_.push_back(map_camIndexToNumber[imageId]);
					}
				}
			}

			// The same K matrix is used by all the camera
			const Mat3 camera_matrix_ = vec_intrinsic_groups_[0].camera_matrix;

			// Create residuals for each observation in the bundle adjustment problem. The
			// parameters for cameras and points are added automatically.
			ceres::Problem problem;
			// Set a LossFunction to be less penalized by false measurements
			//  - set it to NULL if you don't want use a lossFunction.
			ceres::LossFunction * p_LossFunction = new ceres::HuberLoss(Square(2.0));
			for (size_t k = 0; k < ba_problem.num_observations(); ++k) {
				// Each Residual block takes a point and a camera as input and outputs a 2
				// dimensional residual. Internally, the cost function stores the observed
				// image location and compares the reprojection against the observation.

				ceres::CostFunction* cost_function =
					new ceres::AutoDiffCostFunction<PinholeReprojectionError_Rt, 2, 6, 3>(
					new PinholeReprojectionError_Rt(
					&ba_problem.observations()[2 * k + 0], camera_matrix_(0, 0)));

				problem.AddResidualBlock(cost_function,
					p_LossFunction,
					ba_problem.mutable_camera_for_observation(k),
					ba_problem.mutable_point_for_observation(k));
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
					// No sparse backend for Ceres.
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
			MVG_INFO << summary.FullReport() << std::endl;

			// If no error, get back refined parameters
			if (summary.IsSolutionUsable())
			{
				// Get back 3D points
				i = 0;
				for (std::vector<Vec3>::iterator iter = vec_all_scenes.begin();
					iter != vec_all_scenes.end(); ++iter, ++i)
				{
					const double * pt = ba_problem.mutable_points() + i * 3;
					Vec3 & point_3d = *iter;
					point_3d = Vec3(pt[0], pt[1], pt[2]);
				}
				exportToPly(vec_all_scenes, mvg::utils::create_filespec(out_dir_, "raw_pointCloud_BA_RT_Xi", "ply"));

				// Get back camera
				i = 0;
				for (MapCamera::iterator iter = map_camera.begin();
					iter != map_camera.end(); ++iter, ++i)
				{
					const double * cam = ba_problem.mutable_cameras() + i * 6;
					Mat3 R;
					// angle axis to rotation matrix
					ceres::AngleAxisToRotationMatrix(cam, R.data());

					Vec3 t(cam[3], cam[4], cam[5]);

					// Update the camera
					Mat3 K = iter->second.camera_matrix_;
					PinholeCamera & sCam = iter->second;
					sCam = PinholeCamera(K, R, t);
				}

	{
		//-- Export Second bundle adjustment statistics
		if (is_html_report_)
		{
			std::ostringstream os;
			os << "Second Bundle Adjustment (Ri, Ti, Xjs) statistics.";
			html_doc_stream_->pushInfo("<hr>");
			html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

			os.str("");
			os << "-------------------------------" << "<br>"
				<< "-- #tracks: " << map_tracks_selected.size() << ".<br>"
				<< "-- #observation: " << ba_problem.num_observations_ << ".<br>"
				<< "-- residual mean (RMSE): " << std::sqrt(summary.final_cost / ba_problem.num_observations_) << ".<br>"
				<< "-- Nb Steps required until convergence : " << summary.num_successful_steps + summary.num_unsuccessful_steps << ".<br>"
				<< "-------------------------------" << "<br>";
			html_doc_stream_->pushInfo(os.str());
		}

		MVG_INFO << "\n"
			<< "-------------------------------" << "\n"
			<< "-- #tracks: " << map_tracks_selected.size() << ".\n"
			<< "-- #observation: " << ba_problem.num_observations_ << ".\n"
			<< "-- residual mean (RMSE): " << std::sqrt(summary.final_cost / ba_problem.num_observations_) << ".\n"
			<< "-- Nb Steps required until convergence : " << summary.num_successful_steps + summary.num_unsuccessful_steps << ".\n"
			<< "-------------------------------" << std::endl;

	}
			}
		}

		void GlobalReconstructionEngine::ColorizeTracks(
			const MapTracks & map_tracks,
			std::vector<Vec3> & vec_tracks_color) const
		{
			// Colorize each track
			//  Start with the most representative image
			//    and iterate to provide a color to each 3D point

			{
				ControlProgressDisplay my_progress_bar(map_tracks.size(),
					std::cout,
					"\nCompute scene structure color\n");

				vec_tracks_color.resize(map_tracks.size());

				//Build a list of contiguous index for the trackIds
				std::map<size_t, size_t> trackIds_to_contiguousIndexes;
				size_t cpt = 0;
				for (mvg::tracking::MapTracks::const_iterator it = map_tracks.begin();
					it != map_tracks.end(); ++it, ++cpt)
				{
					trackIds_to_contiguousIndexes[it->first] = cpt;
				}

				// The track list that will be colored (point removed during the process)
				mvg::tracking::MapTracks mapTrackToColor(map_tracks);
				while (!mapTrackToColor.empty())
				{
					// Find the most representative image
					//  a. Count the number of visible point for each image
					//  b. Sort to find the most representative image

					std::map<size_t, size_t> map_IndexCardinal; // ImageIndex, Cardinal
					for (mvg::tracking::MapTracks::const_iterator
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
					std::vector< mvg::utils::SortIndexPacketDescend< size_t, size_t> > packet_vec(vec_cardinal.size());
					mvg::utils::SortIndexHelper(packet_vec, &vec_cardinal[0]);

					//First index is the image with the most of matches
					std::map<size_t, size_t>::const_iterator iterTT = map_IndexCardinal.begin();
					std::advance(iterTT, packet_vec[0].index);
					const size_t indexImage = iterTT->first;
					mvg::image::Image<mvg::image::RGBColor> image;
					readImage(
						mvg::utils::create_filespec(
						image_path_,
						mvg::utils::basename_part(vec_camera_image_names_[indexImage].image_name),
						mvg::utils::extension_part(vec_camera_image_names_[indexImage].image_name)).c_str(), &image);

					// Iterate through the track
					std::set<size_t> set_toRemove;
					for (mvg::tracking::MapTracks::const_iterator
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
							const mvg::image::RGBColor color = image(feat.y(), feat.x());

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
			}
		} // namespace mvg
