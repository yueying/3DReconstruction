#ifndef FBLIB_FEATURE_GEOMETRIC_FILTER_H_
#define FBLIB_FEATURE_GEOMETRIC_FILTER_H_

#include <vector>
#include <map>

#include "fblib/feature/features.h"
#include "fblib/utils//file_system.h"
#include "fblib/utils/progress.h"

namespace fblib{
	namespace feature{
		template <typename FeatureT>
		class ImageCollectionGeometricFilter
		{
		public:
			ImageCollectionGeometricFilter(){}

			/**	导入特征  
			 * \param [in,out]	file_names	输入文件名
			 * \param [in,out]	match_dir	数据存储目录
			 */
			bool LoadData(const std::vector<std::string> &file_names, const std::string &match_dir) 
			{
				bool is_ok = true;
				for (size_t j = 0; j < file_names.size(); ++j)  {
					// 导入第j张图片的特征数据
					const std::string feat_filename = fblib::utils::create_filespec(match_dir,
						fblib::utils::basename_part(file_names[j]), "feat");
					is_ok &= LoadFeatsFromFile(feat_filename, map_features[j]);
				}
				return is_ok;
			}

			/// Filter all putative correspondences according the templated geometric filter
			template <typename GeometricFilterT>
			void Filter(
				const GeometricFilterT &geometric_filter,
				PairWiseMatches &map_putatives_matches_pair, // putative correspondences to filter
				PairWiseMatches &map_geometric_matches,
				const std::vector<std::pair<size_t, size_t> > &vec_images_size) const
			{
				ControlProgressDisplay my_progress_bar(map_putatives_matches_pair.size());

#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
				for (size_t i = 0; i < map_putatives_matches_pair.size(); ++i)
				{
					PairWiseMatches::const_iterator iter = map_putatives_matches_pair.begin();
					std::advance(iter, i);

					const size_t i_index = iter->first.first;
					const size_t j_index = iter->first.second;
					const std::vector<IndexedMatch> &vec_putative_matches = iter->second;

					//导入第i和j图像的特征
					typename std::map<size_t, std::vector<FeatureT> >::const_iterator iter_i_feats = map_features.begin();
					typename std::map<size_t, std::vector<FeatureT> >::const_iterator iter_j_feats = map_features.begin();
					std::advance(iter_i_feats, i_index);
					std::advance(iter_j_feats, j_index);
					const std::vector<FeatureT> & kpSetI = iter_i_feats->second;
					const std::vector<FeatureT> & kpSetJ = iter_j_feats->second;

					//-- Copy point to array in order to estimate fundamental matrix :
					const size_t n = vec_putative_matches.size();
					Mat xI(2, n), xJ(2, n);

					for (size_t i = 0; i < vec_putative_matches.size(); ++i)  {
						const FeatureT & left_img = kpSetI[vec_putative_matches[i]._i];
						const FeatureT & right_img = kpSetJ[vec_putative_matches[i]._j];
						xI.col(i) = Vec2f(left_img.coords()).cast<double>();
						xJ.col(i) = Vec2f(right_img.coords()).cast<double>();
					}

					//-- Apply the geometric filter
	  {
		  std::vector<size_t> vec_inliers;
		  // Use a copy in order to copy use internal functor parameters
		  // and use it safely in multi-thread environment
		  GeometricFilterT filter = geometric_filter;
		  filter.Fit(xI, vec_images_size[i_index], xJ, vec_images_size[j_index], vec_inliers);

		  if (!vec_inliers.empty())
		  {
			  std::vector<IndexedMatch> vec_filtered_matches;
			  vec_filtered_matches.reserve(vec_inliers.size());
			  for (size_t i = 0; i < vec_inliers.size(); ++i)  {
				  vec_filtered_matches.push_back(vec_putative_matches[vec_inliers[i]]);
			  }
#ifdef USE_OPENMP
#pragma omp critical
#endif
		  {
			  map_geometric_matches[std::make_pair(i_index, j_index)] = vec_filtered_matches;
		  }
		  }
	  }
					++my_progress_bar;
				}
			}

		private:
			std::map<size_t, std::vector<FeatureT> > map_features;//!<每张图片对应的特征
		};

	}
}


#endif // FBLIB_FEATURE_GEOMETRIC_FILTER_H_