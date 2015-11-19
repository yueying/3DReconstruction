// Implementation of [1] an efficient algorithm to compute track from pairwise
//  correspondences.
//
//  [1] Pierre Moulon and Pascal Monasse,
//    "Unordered feature tracking made fast and easy" CVMP 2012.
//
// It tracks the position of features along the series of image from pairwise
//  correspondences.
//
// From map< [imageI,ImageJ], [indexed matches array] > it builds tracks.
//
// Usage :
//  PairWiseMatches map_matches;
//  PairedIndexedMatchImport(match_file, map_matches); // Load series of pairwise matches
//  //---------------------------------------
//  // Compute tracks from matches
//  //---------------------------------------
//  TracksBuilder tracks_builder;
//  tracking::MapTracks map_tracks;
//  tracks_builder.Build(map_matches); // Build: Efficient fusion of correspondences
//  tracks_builder.Filter();           // Filter: Remove track that have conflict
//  tracks_builder.ExportToSTL(map_tracks); // Build tracks with STL compliant type
//

#ifndef FBLIB_TRACKER_TRACKS_H_
#define FBLIB_TRACKER_TRACKS_H_

#include <algorithm>
#include <iostream>
#include <functional>
#include <vector>
#include <set>
#include <map>
#include <memory>

#include "lemon/list_graph.h"
#include "lemon/unionfind.h"
using namespace lemon;

#include "fblib/feature/indexed_match.h"

namespace fblib  {
	namespace tracking{
		using namespace fblib::feature;

		/**
		 * \brief	构建一个轻量级的map，通过vector<std::pari<T1,T2> >进行构建
		 * 			方便进行插入运算
		 *
		 * \tparam	T1	Generic type parameter.
		 * \tparam	T2	Generic type parameter.
		 */
		template<typename T1, typename T2>
		class flat_pair_map
		{
			typedef std::pair<T1, T2> P;
		public:
			typedef typename std::vector< P >::iterator iterator;

			typename std::vector< P >::iterator find(const T1 & val)  {
				return std::lower_bound(m_vec.begin(), m_vec.end(), val, SuperiorToFirst);
			}

			T2 & operator[](const T1 & val) {
				return std::lower_bound(m_vec.begin(), m_vec.end(), val, SuperiorToFirst)->second;
			}

			void sort()  { std::sort(m_vec.begin(), m_vec.end(), SortPairAscend); }
			void push_back(const P & val)  { m_vec.push_back(val); }
			void clear()  { m_vec.clear(); }
			void reserve(size_t count)  { m_vec.reserve(count); }

		private:
			std::vector< P > m_vec;

			static bool SortPairAscend(const P &a, const P &b) { return a.first < b.first; }
			static bool SuperiorToFirst(const P &a, const T1 &b) { return a.first < b; }
		};

		//  结构用来存储跟踪，为对应{ImageId,FeatureId}的集合
		typedef std::map<size_t, size_t> SubmapTrack;
		// 表示跟踪的集合{TrackId, SubmapTrack}
		typedef std::map< size_t, SubmapTrack > MapTracks;
		/**	构建tracks
		 */
		struct TracksBuilder
		{
			typedef std::pair<size_t, size_t> IndexedFeaturePair;//!< 匹配的特征对
			typedef lemon::ListDigraph::NodeMap<size_t> IndexMap;
			typedef lemon::UnionFindEnum< IndexMap > UnionFindObject;

			typedef flat_pair_map< lemon::ListDigraph::Node, IndexedFeaturePair> MapNodeIndex;
			typedef flat_pair_map< IndexedFeaturePair, lemon::ListDigraph::Node > MapIndexNode;

			lemon::ListDigraph g; //!<图节点容器，存放图节点
			MapNodeIndex reverse_my_Map; //Node to index map
			std::shared_ptr<IndexMap> index;
			std::shared_ptr<UnionFindObject> myTracksUF;

			const UnionFindObject & getUnionFindEnum() const { return *myTracksUF; }
			const MapNodeIndex & getReverseMap() const { return reverse_my_Map; }

			/// Build tracks for a given series of pairWise matches
			bool Build(const PairWiseMatches &  map_pair_wise_matches)
			{
				typedef std::set<IndexedFeaturePair> SetIndexedPair;
				SetIndexedPair myset;
				for (PairWiseMatches::const_iterator iter = map_pair_wise_matches.begin();
					iter != map_pair_wise_matches.end();
					++iter)
				{
					const size_t & I = iter->first.first;
					const size_t & J = iter->first.second;
					const std::vector<IndexedMatch> & vec_filtered_matches = iter->second;
					// We have correspondences between I and J image index.

					for (size_t k = 0; k < vec_filtered_matches.size(); ++k)
					{
						// Look if one of the feature already belong to a track :
						myset.insert(std::make_pair(I, vec_filtered_matches[k]._i));
						myset.insert(std::make_pair(J, vec_filtered_matches[k]._j));
					}
				}

				// Build the node indirection for each referenced feature
				MapIndexNode my_Map;
				my_Map.reserve(myset.size());
				reverse_my_Map.reserve(myset.size());
				for (SetIndexedPair::const_iterator iter = myset.begin();
					iter != myset.end();
					++iter)
				{
					lemon::ListDigraph::Node node = g.addNode();
					my_Map.push_back(std::make_pair(*iter, node));
					reverse_my_Map.push_back(std::make_pair(node, *iter));
				}

				// Sort the flat_pair_map
				my_Map.sort();
				reverse_my_Map.sort();

				// Add the element of myset to the UnionFind insert method.
				index = std::shared_ptr<IndexMap>(new IndexMap(g));
				myTracksUF = std::shared_ptr<UnionFindObject>(new UnionFindObject(*index));
				for (lemon::ListDigraph::NodeIt it(g); it != INVALID; ++it) {
					myTracksUF->insert(it);
				}

				// Make the union according the pair matches
				for (PairWiseMatches::const_iterator iter = map_pair_wise_matches.begin();
					iter != map_pair_wise_matches.end();
					++iter)
				{
					const size_t & I = iter->first.first;
					const size_t & J = iter->first.second;
					const std::vector<IndexedMatch> & vec_filtered_matches = iter->second;
					// We have correspondences between I and J image index.

					for (size_t k = 0; k < vec_filtered_matches.size(); ++k)
					{
						IndexedFeaturePair pairI = std::make_pair(I, vec_filtered_matches[k]._i);
						IndexedFeaturePair pairJ = std::make_pair(J, vec_filtered_matches[k]._j);
						myTracksUF->join(my_Map[pairI], my_Map[pairJ]);
					}
				}
				return false;
			}

			/// Remove bad tracks, conflict tracks (many times the same image index in a track)
			bool Filter(size_t nLengthSupTo = 2)
			{
				// Remove bad tracks (shorter, conflicts (Many times the same image index)...)

				// Remove tracks that have a conflict (many times the same image index)
				std::set<int> set_classToErase;
				for (lemon::UnionFindEnum< IndexMap >::ClassIt cit(*myTracksUF); cit != INVALID; ++cit) {
					size_t cpt = 0;
					std::set<size_t> myset;
					for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*myTracksUF, cit); iit != INVALID; ++iit) {
						myset.insert(reverse_my_Map[iit].first);
						++cpt;
					}
					if (myset.size() != cpt || myset.size() < nLengthSupTo)
					{
						set_classToErase.insert(cit.operator int());
					}
				}
				for_each(set_classToErase.begin(), set_classToErase.end(),
					std::bind1st(std::mem_fun(&UnionFindObject::eraseClass), myTracksUF.get()));
				return false;
			}

			/// Remove the pair that have too few correspondences.
			bool FilterPairWiseMinimumMatches(size_t min_matches_occurences, bool is_verbose = false)
			{
				std::vector<size_t> vec_tracks_to_remove;
				std::map< size_t, std::set<size_t> > map_tracks_id_per_images;

				//-- Count the number of track per image Id
				for (lemon::UnionFindEnum< IndexMap >::ClassIt cit(*myTracksUF); cit != INVALID; ++cit) {
					size_t trackId = cit.operator int();
					for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*myTracksUF, cit); iit != INVALID; ++iit) {
						const MapNodeIndex::iterator iterTrackValue = reverse_my_Map.find(iit);
						const IndexedFeaturePair & currentPair = iterTrackValue->second;
						map_tracks_id_per_images[currentPair.first].insert(trackId);
					}
				}

				//-- Compute cross images matches
				for (std::map<size_t, std::set<size_t> >::const_iterator iter = map_tracks_id_per_images.begin();
					iter != map_tracks_id_per_images.end();
					++iter)
				{
					const std::set<size_t> & setA = iter->second;
					for (std::map<size_t, std::set<size_t> >::const_iterator iter2 = iter;
						iter2 != map_tracks_id_per_images.end();  ++iter2)
					{
						const std::set<size_t> & setB = iter2->second;
						std::vector<size_t> inter;

						std::set_intersection(setA.begin(), setA.end(), setB.begin(), setB.end(), back_inserter(inter));

						if (inter.size() < min_matches_occurences)
							copy(inter.begin(), inter.end(), back_inserter(vec_tracks_to_remove));
					}
				}
				std::sort(vec_tracks_to_remove.begin(), vec_tracks_to_remove.end());
				std::vector<size_t>::iterator it = std::unique(vec_tracks_to_remove.begin(), vec_tracks_to_remove.end());
				vec_tracks_to_remove.resize(std::distance(vec_tracks_to_remove.begin(), it));
				if (is_verbose)
					std::cout << std::endl << std::endl << vec_tracks_to_remove.size()
					<< " Tracks will be removed" << std::endl;
				std::for_each(vec_tracks_to_remove.begin(), vec_tracks_to_remove.end(),
					std::bind1st(std::mem_fun(&UnionFindObject::eraseClass), myTracksUF.get()));
				return false;
			}

			bool ExportToStream(std::ostream & os)
			{
				size_t cpt = 0;
				for (lemon::UnionFindEnum< IndexMap >::ClassIt cit(*myTracksUF); cit != INVALID; ++cit) {
					os << "Class: " << cpt++ << std::endl;
					size_t cptTrackLength = 0;
					for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*myTracksUF, cit); iit != INVALID; ++iit) {
						++cptTrackLength;
					}
					os << "\t" << "track length: " << cptTrackLength << std::endl;

					for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*myTracksUF, cit); iit != INVALID; ++iit) {
						os << reverse_my_Map[iit].first << "  " << reverse_my_Map[iit].second << std::endl;
					}
				}
				return os.good();
			}

			/// Return the number of connected set in the UnionFind structure (tree forest)
			size_t NbTracks() const
			{
				size_t cpt = 0;
				for (lemon::UnionFindEnum< IndexMap >::ClassIt cit(*myTracksUF); cit != INVALID; ++cit)
					++cpt;
				return cpt;
			}

			/// Export tracks as a map (each entry is a sequence of imageId and featureIndex):
			///  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
			void ExportToSTL(MapTracks & map_tracks)
			{
				map_tracks.clear();

				size_t cptClass = 0;
				for (lemon::UnionFindEnum< IndexMap >::ClassIt cit(*myTracksUF); cit != INVALID; ++cit, ++cptClass) {
					std::pair<MapTracks::iterator, bool> ret =
						map_tracks.insert(std::pair<size_t, SubmapTrack >(cptClass, SubmapTrack()));
					MapTracks::iterator iterN = ret.first;

					for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*myTracksUF, cit); iit != INVALID; ++iit) {
						const MapNodeIndex::iterator iterTrackValue = reverse_my_Map.find(iit);
						const IndexedFeaturePair & currentPair = iterTrackValue->second;

						iterN->second[currentPair.first] = currentPair.second;
					}
				}
			}
		};

		struct TracksUtilsMap
		{
			/// Return the tracks that are in common to the set_image_index indexes.
			static bool GetTracksInImages(
				const std::set<size_t> & set_image_index,
				const MapTracks & map_tracks_in,
				MapTracks & map_tracks_out)
			{
				map_tracks_out.clear();

				// Go along the tracks
				for (MapTracks::const_iterator tracks_iter = map_tracks_in.begin();
					tracks_iter != map_tracks_in.end(); ++tracks_iter)  {

					// If the track contain one of the provided index save the point of the track
					std::map<size_t, size_t> map_temp;
					for (std::set<size_t>::const_iterator iterIndex = set_image_index.begin();
						iterIndex != set_image_index.end(); ++iterIndex)
					{
						SubmapTrack::const_iterator iterSearch = tracks_iter->second.find(*iterIndex);
						if (iterSearch != tracks_iter->second.end())
							map_temp[iterSearch->first] = iterSearch->second;
					}

					if (!map_temp.empty() && map_temp.size() == set_image_index.size())
						map_tracks_out.insert(make_pair(tracks_iter->first, map_temp));
				}
				return !map_tracks_out.empty();
			}

			/// Return the tracksId as a set (sorted increasing)
			static void GetTracksIdVector(
				const MapTracks & map_tracks,
				std::set<size_t> * set_tracksIds)
			{
				set_tracksIds->clear();
				for (MapTracks::const_iterator tracks_iter = map_tracks.begin();
					tracks_iter != map_tracks.end(); ++tracks_iter)
				{
					set_tracksIds->insert(tracks_iter->first);
				}
			}

			/// Get feature index PerView and TrackId
			static bool GetFeatIndexPerViewAndTrackId(
				const MapTracks & map_tracks,
				const std::set<size_t> & set_trackId,
				size_t nImageIndex,
				std::vector<size_t> * pvec_featIndex)
			{
				for (MapTracks::const_iterator tracks_iter = map_tracks.begin();
					tracks_iter != map_tracks.end(); ++tracks_iter)
				{
					size_t trackId = tracks_iter->first;
					if (set_trackId.find(trackId) != set_trackId.end())
					{
						//try to find imageIndex
						const SubmapTrack & map_ref = tracks_iter->second;
						SubmapTrack::const_iterator iterSearch = map_ref.find(nImageIndex);
						if (iterSearch != map_ref.end())
						{
							pvec_featIndex->push_back(iterSearch->second);
						}
					}
				}
				return !pvec_featIndex->empty();
			}

			struct FunctorMapFirstEqual : public std::unary_function < std::map<size_t, std::map<size_t, size_t> >, bool >
			{
				size_t id;
				FunctorMapFirstEqual(size_t val) :id(val){};
				bool operator()(const std::pair<size_t, std::map<size_t, size_t> > & val) {
					return (id == val.first);
				}
			};

			/// Convert a trackId to a vector of indexed Matches.
			/// The input tracks must be compound of only two images index.
			/// Be careful image index are sorted (increasing order)
			/// Only track index contained in the filter vector are kept.
			static void TracksToIndexedMatches(const MapTracks & map_tracks,
				const std::vector<size_t> & vec_filterIndex,
				std::vector<IndexedMatch> * pvec_index)
			{

				std::vector<IndexedMatch> & vec_indexref = *pvec_index;
				vec_indexref.clear();
				for (size_t i = 0; i < vec_filterIndex.size(); ++i)
				{
					MapTracks::const_iterator itF =
						find_if(map_tracks.begin(), map_tracks.end(), FunctorMapFirstEqual(vec_filterIndex[i]));
					const SubmapTrack & map_ref = itF->second;
					SubmapTrack::const_iterator iter = map_ref.begin();
					size_t indexI = iter->second;
					++iter;
					size_t indexJ = iter->second;
					vec_indexref.push_back(IndexedMatch(indexI, indexJ));
				}
			}

			/// Return the occurrence of tracks length.
			static void TracksLength(const MapTracks & map_tracks,
				std::map<size_t, size_t> & map_Occurence_TrackLength)
			{
				for (MapTracks::const_iterator tracks_iter = map_tracks.begin();
					tracks_iter != map_tracks.end(); ++tracks_iter)
				{
					size_t trLength = tracks_iter->second.size();
					if (map_Occurence_TrackLength.end() ==
						map_Occurence_TrackLength.find(trLength))
					{
						map_Occurence_TrackLength[trLength] = 1;
					}
					else
					{
						map_Occurence_TrackLength[trLength] += 1;
					}
				}
			}

			/// Return a set containing the image Id considered in the tracks container.
			static void ImageIdInTracks(const MapTracks & map_tracks,
				std::set<size_t> & set_imagesId)
			{
				for (MapTracks::const_iterator tracks_iter = map_tracks.begin();
					tracks_iter != map_tracks.end(); ++tracks_iter)
				{
					const SubmapTrack & map_ref = tracks_iter->second;
					for (SubmapTrack::const_iterator iter = map_ref.begin();
						iter != map_ref.end();
						++iter)
					{
						set_imagesId.insert(iter->first);
					}
				}
			}
		};

	} // namespace tracking
} // namespace fblib

#endif // FBLIB_TRACKER_TRACKS_H_
