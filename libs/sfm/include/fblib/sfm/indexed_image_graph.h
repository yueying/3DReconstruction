#ifndef FBLIB_SFM_INDEXED_IMAGE_GRAPH_H_
#define FBLIB_SFM_INDEXED_IMAGE_GRAPH_H_

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <string>

#include "lemon/list_graph.h"
using namespace lemon;
#include "fblib/feature/indexed_match.h"
using namespace fblib::feature;

namespace fblib  {
	namespace sfm  {
		using namespace std;

		// Structure used to keep information of an image graph :
		//  - A graph (connection between nodes
		//  - Node => linked to String (the name of the image)
		//  - EdgeMap => Number of point connection between the source and target
		//
		struct IndexedImageGraph
		{
			typedef lemon::ListGraph GraphT;
			typedef std::map<size_t, GraphT::Node> map_Size_t_Node;
			typedef GraphT::NodeMap<size_t> map_NodeMapIndex;
			typedef GraphT::NodeMap<std::string> map_NodeMapName;
			typedef GraphT::EdgeMap<size_t> map_EdgeMap;

			GraphT g;
			map_Size_t_Node map_size_t_to_node; // Original image index to graph node
			auto_ptr<map_NodeMapIndex> map_nodeMapIndex; // Association of data to graph Node
			auto_ptr<map_NodeMapName> map_codeMapName; // Association of data to graph Node
			auto_ptr<map_EdgeMap> map_edgeMap; // Number of point matches between the source and the target

			IndexedImageGraph(const PairWiseMatches & map_indexed_matches,
				const std::vector<string> &vec_fileNames)
			{
				map_nodeMapIndex = auto_ptr<map_NodeMapIndex>(new map_NodeMapIndex(g));
				map_codeMapName = auto_ptr<map_NodeMapName>(new map_NodeMapName(g));
				map_edgeMap = auto_ptr<map_EdgeMap>(new map_EdgeMap(g));

				//A-- Compute the number of node we need
				set<size_t> setNodes;
				for (PairWiseMatches::const_iterator iter = map_indexed_matches.begin();
					iter != map_indexed_matches.end();
					++iter)
				{
					setNodes.insert(iter->first.first);
					setNodes.insert(iter->first.second);
				}

				//B-- Create a node graph for each element of the set
				for (set<size_t>::const_iterator iter = setNodes.begin();
					iter != setNodes.end();
					++iter)
				{
					map_size_t_to_node[*iter] = g.addNode();
					(*map_nodeMapIndex)[map_size_t_to_node[*iter]] = *iter;
					(*map_codeMapName)[map_size_t_to_node[*iter]] = vec_fileNames[*iter];
				}

				//C-- Add weighted edges from the "map_indexed_matches" object
				for (PairWiseMatches::const_iterator iter = map_indexed_matches.begin();
					iter != map_indexed_matches.end();
					++iter)
				{
					const std::vector<IndexedMatch> & vec_filtered_matches = iter->second;
					if (vec_filtered_matches.size() > 0)
					{
						const size_t i = iter->first.first;
						const size_t j = iter->first.second;
						GraphT::Edge edge = g.addEdge(map_size_t_to_node[i], map_size_t_to_node[j]);
						(*map_edgeMap)[edge] = vec_filtered_matches.size();
					}
				}
			}
		};

	} // namespace sfm
} // namespace fblib

#endif // FBLIB_SFM_INDEXED_IMAGE_GRAPH_H_
