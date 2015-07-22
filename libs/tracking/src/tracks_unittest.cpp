
#include "testing.h"

#include "fblib/tracking/tracks.h"
#include "fblib/feature/indexed_match.h"
using namespace fblib::tracking;
using namespace fblib::feature;

#include <vector>
#include <utility>

TEST(Tracks, Simple) {

  /*
  A    B    C
  0 -> 0 -> 0
  1 -> 1 -> 6
  2 -> 3
  */

  // Create the input pairwise correspondences
  PairWiseMatches map_pairwisematches;

  IndexedMatch testAB[] = {IndexedMatch(0,0), IndexedMatch(1,1), IndexedMatch(2,3)};
  IndexedMatch testBC[] = {IndexedMatch(0,0), IndexedMatch(1,6)};


  std::vector<IndexedMatch> ab(testAB, testAB+3);
  std::vector<IndexedMatch> bc(testBC, testBC+2);
  const int A = 0;
  const int B = 1;
  const int C = 2;
  map_pairwisematches[ std::make_pair(A,B) ] = ab;
  map_pairwisematches[ std::make_pair(B,C) ] = bc;

  //-- Build tracks using the interface tracksbuilder
  TracksBuilder trackBuilder;
  trackBuilder.Build( map_pairwisematches );

  trackBuilder.ExportToStream(std::cout);

  MapTracks map_tracks;
  trackBuilder.ExportToSTL(map_tracks);

  //-------------------
  // Unit Test check
  //-------------------

  //0, {(0,0) (1,0) (2,0)}
  //1, {(0,1) (1,1) (2,6)}
  //2, {(0,2) (1,3)}
  const std::pair<size_t,size_t> GT_Tracks[] =
    {std::make_pair(0,0), std::make_pair(1,0), std::make_pair(2,0),
     std::make_pair(0,1), std::make_pair(1,1), std::make_pair(2,6),
     std::make_pair(0,2), std::make_pair(1,3)};

  EXPECT_EQ(3, map_tracks.size());
  size_t cpt = 0, i = 0;
  for (MapTracks::const_iterator iterT = map_tracks.begin();
    iterT != map_tracks.end();
    ++iterT, ++i)
  {
	  EXPECT_EQ(i, iterT->first);
    for (SubmapTrack::const_iterator iter = iterT->second.begin();
      iter != iterT->second.end();
      ++iter)
    {
		EXPECT_TRUE(GT_Tracks[cpt] == std::make_pair(iter->first, iter->second));
      ++cpt;
    }
  }
}

TEST(Tracks, filter_3viewAtLeast) {

  /*
  A    B    C
  0 -> 0 -> 0
  1 -> 1 -> 6
  2 -> 3
  */

  // Create the input pairwise correspondences
  PairWiseMatches map_pairwisematches;

  IndexedMatch testAB[] = {IndexedMatch(0,0), IndexedMatch(1,1), IndexedMatch(2,3)};
  IndexedMatch testBC[] = {IndexedMatch(0,0), IndexedMatch(1,6)};


  std::vector<IndexedMatch> ab(testAB, testAB+3);
  std::vector<IndexedMatch> bc(testBC, testBC+2);
  const int A = 0;
  const int B = 1;
  const int C = 2;
  map_pairwisematches[ std::make_pair(A,B) ] = ab;
  map_pairwisematches[ std::make_pair(B,C) ] = bc;

  //-- Build tracks using the interface tracksbuilder
  TracksBuilder trackBuilder;
  trackBuilder.Build( map_pairwisematches );
  EXPECT_EQ(3, trackBuilder.NbTracks());
  trackBuilder.Filter(3);
  EXPECT_EQ(2, trackBuilder.NbTracks());
}

TEST(Tracks, Conflict) {

  /*
  A    B    C
  0 -> 0 -> 0
  1 -> 1 -> 6
  2 -> 3 -> 2}
       3 -> 8 } This track must be deleted, index 3 appears two times
  */

  // Create the input pairwise correspondences
  PairWiseMatches map_pairwisematches;

  IndexedMatch testAB[] = {IndexedMatch(0,0), IndexedMatch(1,1), IndexedMatch(2,3)};
  IndexedMatch testBC[] = {IndexedMatch(0,0), IndexedMatch(1,6), IndexedMatch(3,2), IndexedMatch(3,8)};

  std::vector<IndexedMatch> ab(testAB, testAB+3);
  std::vector<IndexedMatch> bc(testBC, testBC+4);
  const int A = 0;
  const int B = 1;
  const int C = 2;
  map_pairwisematches[ std::make_pair(A,B) ] = ab;
  map_pairwisematches[ std::make_pair(B,C) ] = bc;

  //-- Build tracks using the interface tracksbuilder
  TracksBuilder trackBuilder;
  trackBuilder.Build( map_pairwisematches );

  trackBuilder.ExportToStream(std::cout); //Export to console
  trackBuilder.Filter(); // Key feature tested here to kill the conflicted track

  MapTracks map_tracks;
  trackBuilder.ExportToSTL(map_tracks);

  //-------------------
  // Unit Test check
  //-------------------

  //0, {(0,0) (1,0) (2,0)}
  //1, {(0,1) (1,1) (2,6)}
  const std::pair<size_t,size_t> GT_Tracks[] =
    {std::make_pair(0,0), std::make_pair(1,0), std::make_pair(2,0),
     std::make_pair(0,1), std::make_pair(1,1), std::make_pair(2,6)};

  EXPECT_EQ(2, map_tracks.size());
  size_t cpt = 0, i = 0;
  for (MapTracks::const_iterator iterT = map_tracks.begin();
    iterT != map_tracks.end();
    ++iterT, ++i)
  {
	  EXPECT_EQ(i, iterT->first);
    for (SubmapTrack::const_iterator iter = iterT->second.begin();
      iter != iterT->second.end();
      ++iter)
    {
		EXPECT_TRUE(GT_Tracks[cpt] == std::make_pair(iter->first, iter->second));
      ++cpt;
    }
  }
}

