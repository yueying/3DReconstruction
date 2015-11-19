#ifndef FBLIB_SFM_GLOBAL_SFM_ENGINE_H
#define FBLIB_SFM_GLOBAL_SFM_ENGINE_H

#include "fblib/sfm/link_pragmas.h"

#include <memory>

#include "fblib/math/numeric.h"
#include "fblib/utils/html_doc.h"

#include "fblib/feature/feature.h"
#include "fblib/feature/image_list_io_helper.h"
#include "fblib/feature/indexed_match.h"
#include "fblib/camera/pinhole_camera.h"
#include "fblib/sfm/sfm_engine.h"
#include "fblib/sfm/sfm_reconstruction_data.h"
#include "fblib/sfm/triplet_finder.h"
#include "fblib/sfm/global_translations_from_triplets.h"

#include "fblib/tracking/tracks.h"
class ScalePointFeature;
using namespace fblib::tracking;

namespace fblib{
	namespace sfm{

		//------------------
		//-- Bibliography --
		//------------------
		//- [1] "Global Fusion of Relative Motions for Robust, Accurate and Scalable Structure from Motion."
		//- Authors: Pierre MOULON, Pascal MONASSE and Renaud MARLET.
		//- Date: December 2013.
		//- Conference: ICCV.
		//- [2] "Disambiguating visual relations using loop constraints."
		//- Autors: Christopher ZACH, Manfred KLOPSCHITZ and Marc POLLEFEYS.
		//- Date: 2010
		//- Conference : CVPR.
		//- [3] "Efficient and Robust Large-Scale Rotation Averaging"
		//- Authors: Avishek Chatterjee and Venu Madhav Govindu
		//- Date: December 2013.
		//- Conference: ICCV.

		// Implementation of [1]
		// Some points differs from the [1] paper to ease open source port:
		//-- Relative rotation inference:
		//   - only the triplet rejection is performed (in [1] a Bayesian inference on cycle error is performed [2])
		//-- Global rotation computation:
		//   - in [1] they are computed by a sparse least square formulation
		//   - here, can be used:
		//    - a simple dense least square,
		//    - or, the L1 averaging method of [3].
		//-- Linear Programming solver:
		//   - in order to have the best performance it is advised to used the MOSEK LP backend.

		enum ERotationAveragingMethod
		{
			ROTATION_AVERAGING_NONE = 0,
			ROTATION_AVERAGING_L1 = 1,
			ROTATION_AVERAGING_L2 = 2
		};

		class SFM_IMPEXP GlobalReconstructionEngine : public ReconstructionEngine
		{
		public:
			GlobalReconstructionEngine(const std::string & image_path,
				const std::string & matches_path, const std::string & out_dir,
				bool is_html_report = false);

			~GlobalReconstructionEngine();

			virtual bool Process();

			/// Give a color to all the 3D points
			void ColorizeTracks(
				const MapTracks & map_tracks, // tracks to be colorized
				std::vector<Vec3> & vec_tracks_color // output associated color
				) const;

			//--
			// Accessors
			//--

			const reconstructorHelper & refToReconstructorHelper() const
			{
				return reconstructor_data_;
			}

			const fblib::tracking::MapTracks & getTracks() const
			{
				return map_selected_tracks_;
			}

			const std::vector<std::string> getFilenamesVector() const
			{
				return vec_file_names_;
			}

			const std::vector< std::pair<size_t, size_t> > getImagesSize() const
			{
				std::vector< std::pair<size_t, size_t> > vec_imageSize;
				for (std::vector<fblib::feature::CameraInfo>::const_iterator iter_camera_info = vec_camera_image_names_.begin();
					iter_camera_info != vec_camera_image_names_.end();
					iter_camera_info++)
				{
					std::vector<fblib::feature::IntrinsicCameraInfo>::const_iterator it_intrinsic = vec_intrinsic_groups_.begin();
					std::advance(it_intrinsic, iter_camera_info->intrinsic_id);
					vec_imageSize.push_back(std::make_pair(it_intrinsic->width, it_intrinsic->height));
				}
				return vec_imageSize;
			}

			//--
			// TYPEDEF
			//--
			typedef std::map< std::pair<size_t, size_t>, std::pair<Mat3, Vec3> > MapRelativeRT;
			typedef std::map<size_t, fblib::camera::PinholeCamera > MapCamera;

		private:

			/**	读取输入数据（对应点和相机内参）
			 */
			bool ReadInputData();

			bool CleanGraph();

			void ComputeRelativeRt(MapRelativeRT & vec_relatives);

			// Detect and remove the outlier relative rotations
			void RotationInference(MapRelativeRT & map_relatives);

			// Compute the global rotations from relative rotations
			bool ComputeGlobalRotations(
				ERotationAveragingMethod eRotationAveragingMethod,
				const std::map<size_t, size_t> & map_camera_node_to_camera_index,
				const std::map<size_t, size_t> & map_camera_index_to_camera_node,
				const MapRelativeRT & map_relatives,
				std::map<size_t, Mat3> & map_globalR) const;

			// List the triplet of the image connection graph (map_matches_fundamental_)
			void TripletListing(std::vector< Triplet > & vec_triplets) const;

			// Relative rotations inference on relative rotations composition error along 3 length cycles (triplets).
			void TripletRotationRejection(
				std::vector< Triplet > & vec_triplets,
				MapRelativeRT & map_relatives);

			// Compute relative translations over the graph of putative triplets
			void ComputePutativeTranslationEdgesCoverage(
				const std::map<std::size_t, Mat3> & map_globalR,
				const std::vector< Triplet > & vec_triplets,
				std::vector<fblib::sfm::RelativeInfo > & vec_initial_estimates,
				feature::PairWiseMatches & newpairMatches) const;

			// Bundle adjustment : refine structure Xis and camera translations
			void bundleAdjustment_t_Xi(
				MapCamera & map_camera,
				std::vector<Vec3> & vec_all_scenes,
				const MapTracks & map_tracks_selected);

			// Bundle adjustment : refine structure Xis and camera rotations and translations
			void bundleAdjustment_Rt_Xi(
				MapCamera & map_camera,
				std::vector<Vec3> & vec_all_scenes,
				const MapTracks & map_tracks_selected);

		private:
			
			std::vector<std::string> vec_file_names_;//!< 用于重建的图片
			std::vector<fblib::feature::CameraInfo> vec_camera_image_names_;//!<图片名及对应id
			std::vector<fblib::feature::IntrinsicCameraInfo> vec_intrinsic_groups_;//!<图片对应的相机内参
			std::map< size_t, std::vector<fblib::feature::ScalePointFeature> > map_features_; //!< 图片对应的特征

			typedef feature::PairWiseMatches PairWiseMatches;
			PairWiseMatches map_matches_fundamental_; // pairwise matches for Essential matrix model


			//------
			//-- Mapping between camera node Ids and cameraIndex:
			//--------------
			// Graph node Ids mapping to camera Ids
			std::map<size_t, size_t> map_camera_node_to_camera_index; // graph node Id to 0->Ncam
			// camera Ids to graph node Ids
			std::map<size_t, size_t> map_camera_index_to_camera_node; // 0->Ncam correspondance to graph node Id
			//--
			//----

			//-----
			//-- Reconstruction data
			//-----
			// Cameras (Motion)
			MapCamera map_camera_;
			// Structure 
			std::vector<Vec3> vec_all_scenes_;
			// Structure visibility
			MapTracks map_selected_tracks_; // reconstructed track (visibility per 3D point)
			// Scene and structure container (for disk output)
			reconstructorHelper reconstructor_data_;
			//-----


			// -----
			// Reporting ..
			// ----
			bool is_html_report_;
			std::shared_ptr<fblib::utils::HtmlDocumentStream> html_doc_stream_;

		};

	}
} // namespace fblib

#endif // FBLIB_SFM_GLOBAL_SFM_ENGINE_H
