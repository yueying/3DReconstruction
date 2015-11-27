#ifndef MVG_FEATURE_MATCHER_ALL_IN_MEMORY_H_
#define MVG_FEATURE_MATCHER_ALL_IN_MEMORY_H_

#include "mvg/feature/features.h"
#include "mvg/feature/indexed_match_decorator.h"
#include "mvg/feature/matching_filters.h"
#include "mvg/feature/matcher.h"

#include "mvg/utils/file_system.h"
#include "mvg/utils/progress.h"

namespace mvg {
	namespace feature{
		/// Implementation of an Image Collection Matcher
		/// Compute putative matches between a collection of pictures
		/// Spurious correspondences are discarded by using the
		///  a threshold over the distance ratio of the 2 neighbours points.
		///
		template <typename KeypointSetT, typename MatcherT>
		class MatcherAllInMemory : public Matcher
		{
			// Alias to internal stored Feature and Descriptor type
			typedef typename KeypointSetT::FeatureT FeatureT;
			typedef typename KeypointSetT::DescriptorT DescriptorT;
			typedef std::vector<DescriptorT > DescsT; //!< 定义特征描述子的集合
			// Alias to Descriptor value type
			typedef typename DescriptorT::bin_type DescBin_typeT;

		public:
			MatcherAllInMemory(float distRatio) :
				Matcher(),
				distance_ratio(distRatio)
			{
			}

			/**
			 * \brief	Load all features and descriptors in memory
			 *
			 * \param	file_names	List of names of the files.
			 * \param	match_dir 	The match dir where the data are saved.
			 *
			 * \return	true if it succeeds, false if it fails.
			 */
			bool LoadData(
				const std::vector<std::string> & file_names,
				const std::string & match_dir) 
			{
				bool is_ok = true;
				for (size_t j = 0; j < file_names.size(); ++j)  {
					// 导入第j个图像的特征及描述子
					const std::string feat_filename = mvg::utils::create_filespec(match_dir,
						mvg::utils::basename_part(file_names[j]), "feat");
					const std::string desc_filename = mvg::utils::create_filespec(match_dir,
						mvg::utils::basename_part(file_names[j]), "desc");

					is_ok &= LoadFeatsFromFile(feat_filename, map_features[j]);
					is_ok &= LoadDescsFromBinFile(desc_filename, map_descriptors[j]);
				}
				return is_ok;
			}

			void Match(
				const std::vector<std::string> & file_names, // input filenames,
				PairWiseMatches & map_putatives_matches)const // the pairwise photometric corresponding points
			{
#ifdef USE_OPENMP
				std::cout << "Using the OPENMP thread interface" << std::endl;
#endif
				ControlProgressDisplay my_progress_bar(file_names.size()*(file_names.size() - 1) / 2.0);

				for (size_t i = 0; i < file_names.size(); ++i)
				{
					// Load features and descriptors of Inth image
					typename std::map<size_t, std::vector<FeatureT> >::const_iterator iter_FeaturesI = map_features.begin();
					typename std::map<size_t, DescsT >::const_iterator iter_DescriptorI = map_descriptors.begin();
					std::advance(iter_FeaturesI, i);
					std::advance(iter_DescriptorI, i);

					const std::vector<FeatureT> & featureSetI = iter_FeaturesI->second;
					const size_t featureSetI_Size = iter_FeaturesI->second.size();
					const DescBin_typeT * tab0 =
						reinterpret_cast<const DescBin_typeT *>(&iter_DescriptorI->second[0]);

					MatcherT matcher10;
					(matcher10.Build(tab0, featureSetI_Size, DescriptorT::kStaticSize));

#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
					for (int j = i + 1; j < (int)file_names.size(); ++j)
					{
						// Load descriptor of Jnth image
						typename std::map<size_t, std::vector<FeatureT> >::const_iterator iter_FeaturesJ = map_features.begin();
						typename std::map<size_t, DescsT >::const_iterator iter_DescriptorJ = map_descriptors.begin();
						std::advance(iter_FeaturesJ, j);
						std::advance(iter_DescriptorJ, j);

						const std::vector<FeatureT> & featureSetJ = iter_FeaturesJ->second;
						const DescBin_typeT * tab1 =
							reinterpret_cast<const DescBin_typeT *>(&iter_DescriptorJ->second[0]);

						const size_t NNN__ = 2;
						std::vector<int> vec_nIndice10;
						std::vector<typename MatcherT::DistanceType> vec_fDistance10;

						//Find left->right
						matcher10.SearchNeighbours(tab1, featureSetJ.size(), &vec_nIndice10, &vec_fDistance10, NNN__);

						std::vector<IndexedMatch> vec_filtered_matches;
						std::vector<int> vec_NNRatioIndexes;
						DistanceRatioFilter(vec_fDistance10.begin(), // distance start
							vec_fDistance10.end(),  // distance end
							NNN__, // Number of neighbor in iterator sequence (minimum required 2)
							vec_NNRatioIndexes, // output (index that respect Lowe Ratio)
							Square(distance_ratio)); // squared dist ratio due to usage of a squared metric

						for (size_t k = 0; k < vec_NNRatioIndexes.size() - 1 && vec_NNRatioIndexes.size()>0; ++k)
						{
							const size_t index = vec_NNRatioIndexes[k];
							vec_filtered_matches.push_back(
								IndexedMatch(vec_nIndice10[index*NNN__], index));
						}

						// Remove duplicates
						IndexedMatch::getDeduplicated(vec_filtered_matches);

						// Remove matches that have the same X,Y coordinates
						IndexedMatchDecorator<float> matchDeduplicator(vec_filtered_matches, featureSetI, featureSetJ);
						matchDeduplicator.getDeduplicated(vec_filtered_matches);

#ifdef USE_OPENMP
#pragma omp critical
#endif
						{
							map_putatives_matches.insert(make_pair(make_pair(i, j), vec_filtered_matches));
						}

						++my_progress_bar;
					}
				}
			}

		private:
			std::map<size_t, std::vector<FeatureT> > map_features;//!< 每幅图像的所有特征
			std::map<size_t, DescsT > map_descriptors;//!< 每幅图像所有特征描述符
			float distance_ratio;//!<距离的比率用于排除一些虚假的匹配
		};
	}// namespace feature
} // namespace mvg

#endif // MVG_FEATURE_MATCHER_ALL_IN_MEMORY_H_
