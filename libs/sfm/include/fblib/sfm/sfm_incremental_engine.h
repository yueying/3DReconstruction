#ifndef FBLIB_SFM_INCREMENTAL_ENGINE_H
#define FBLIB_SFM_INCREMENTAL_ENGINE_H

#include "fblib/sfm/link_pragmas.h"
#include <memory>

#include "fblib/math/numeric.h"
#include "fblib/sfm/sfm_engine.h"
#include "fblib/sfm/sfm_reconstruction_data.h"
#include "fblib/feature/image_list_io_helper.h"
#include "fblib/feature/features.h"
#include "fblib/tracking/tracks.h"

#include "fblib/utils/histogram.h"
#include "fblib/utils/html_doc.h"

namespace fblib{
	namespace sfm{
		//Estimate E -> So R,t for the first pair
		// Maintain a track list that explain 3D reconstructed scene
		// Add images with Resection with the 3D tracks.
		class SFM_IMPEXP IncrementalReconstructionEngine : public ReconstructionEngine
		{
		public:
			IncrementalReconstructionEngine(const std::string &image_path,
				const std::string &matches_path, const std::string &out_dir,
				bool is_html_report = false);

			~IncrementalReconstructionEngine();

			bool Process();

		private:
			/// Read input data (point correspondences, K matrix)
			bool ReadInputData();

			/**
			 * \brief	选择最好的初始匹配对
			 *
			 * \param [in,out]	initial_pair_index	返回最好的初始匹配对
			 *
			 * \return	true if it succeeds, false if it fails.
			 */
			bool InitialPairChoice(std::pair<size_t, size_t> &initial_pair_index);

			/// Compute the initial 3D seed (First camera t=0; R=Id, second estimated by 5 point algorithm)
			bool MakeInitialPair3D(const std::pair<size_t, size_t> & initial_pair);

			/// List the images that the greatest number of matches to the current 3D reconstruction.
			bool FindImagesWithPossibleResection(std::vector<size_t> & vec_possible_indexes);

			/// Add to the current scene the desired image indexes.
			bool Resection(std::vector<size_t> & vec_possible_indexes);

			/// Add a single Image to the scene and triangulate new possible tracks
			bool Resection(size_t imageIndex);

			/// Discard track with too large residual error
			size_t badTrackRejector(double dPrecision);

		public:
			/// Give a color to all the 3D points
			void ColorizeTracks(std::vector<Vec3> & vec_tracks_color) const;

			const reconstructorHelper & refToReconstructorHelper() const
			{
				return reconstructor_data_;
			}

			/// Bundle adjustment to refine Structure and Motion
			void BundleAdjustment();

			// Return MSE (Mean Square Error) and an histogram of residual values.
			double ComputeResidualsHistogram(Histogram<double> * histo);

			const std::vector<std::string> getFilenamesVector() const
			{
				std::vector<std::string> file_names;
				for (std::vector<fblib::feature::CameraInfo>::const_iterator iter_camera_info = camera_image_names_.begin();
					iter_camera_info != camera_image_names_.end();
					iter_camera_info++)
				{
					file_names.push_back(iter_camera_info->image_name);
				}
				return file_names;
			}

			const fblib::tracking::MapTracks & getTracks() const
			{
				return map_reconstructed_;
			}

			const std::vector< std::pair<size_t, size_t> > getImagesSize() const
			{
				std::vector< std::pair<size_t, size_t> > vec_imageSize;
				for (std::vector<fblib::feature::CameraInfo>::const_iterator iter_camera_info = camera_image_names_.begin();
					iter_camera_info != camera_image_names_.end();
					iter_camera_info++)
				{
					std::vector<fblib::feature::IntrinsicCameraInfo>::const_iterator it_intrinsic = vec_intrinsic_groups_.begin();
					std::advance(it_intrinsic, iter_camera_info->intrinsic_id);
					vec_imageSize.push_back(std::make_pair(it_intrinsic->width, it_intrinsic->height));
				}
				return vec_imageSize;
			}

			void setInitialPair(std::pair<size_t, size_t> initial_pair)
			{
				initial_pair_ = initial_pair;
			}

			void setIfUseBundleAdjustment(bool is_use_bundle_adjustment)
			{
				is_use_bundle_adjustment_ = is_use_bundle_adjustment;
			}

			void setIfRefinePrincipalPointAndRadialDistortion(bool is_refine_point_and_distortion)
			{
				is_refine_point_and_distortion_ = is_refine_point_and_distortion;
			}

		private:

			std::vector<fblib::feature::CameraInfo> camera_image_names_;//!<对应的图像
			std::vector<fblib::feature::IntrinsicCameraInfo> vec_intrinsic_groups_;//!< 内参组合
			std::map< size_t, std::vector<fblib::feature::ScalePointFeature> > map_features_; //!<每张图片对应的特征

			std::map<size_t, size_t> map_intrinsic_id_per_image_id_;//!<图像id对应内参id

			//-- Visibility information
			fblib::feature::PairWiseMatches map_matches_fundamental_; // pairwise matches for Fundamental model
			fblib::tracking::MapTracks map_tracks_; // reconstructed track (visibility per 3D point)

			//-- configuration of the reconstruction
			std::pair<size_t, size_t> initial_pair_;
			bool is_use_bundle_adjustment_;
			bool is_refine_point_and_distortion_; // Boolean used to know if Principal point and Radial disto is refined

			// -----
			// Future reconstructed data
			// ----

			// helper to save reconstructed data (Camera and 3D points)
			reconstructorHelper reconstructor_data_;

			// tracks that are reconstructed during the sequential SfM process
			fblib::tracking::MapTracks map_reconstructed_;

			// Remaining image indexes that could be sequentially added
			std::set<size_t> set_remaining_image_id_;

			//store in which order image have been added
			std::vector<size_t> vec_added_order_;

			// Per camera confidence (A contrario estimated threshold error)
			std::map<size_t, double> map_ac_threshold_;

			/// List of images that belong to a common intrinsic group
			std::map<size_t, std::vector<size_t> > map_images_id_per_intrinsic_group_;
			std::map<size_t, Vec6 > map_intrinsics_per_group_;

			// -----
			// Reporting ..
			// ----
			bool is_html_report_;
			std::shared_ptr<fblib::utils::HtmlDocumentStream> html_doc_stream_;

		};

	} // namespace sfm
} // namespace fblib

#endif // FBLIB_SFM_INCREMENTAL_ENGINE_H
