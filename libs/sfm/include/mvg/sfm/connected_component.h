#ifndef MVG_SFM_CONNECTED_COMPONENT_H_
#define MVG_SFM_CONNECTED_COMPONENT_H_

#include <lemon/connectivity.h>
#include <set>

namespace mvg
{
	namespace sfm
	{
		// Export node of each CC (Connected Component) in a map
		template <typename GraphT>
		std::map<size_t, std::set<lemon::ListGraph::Node> >  ExportGraphToMapSubgraphs(
			const GraphT & g)
		{
			typedef lemon::ListGraph::NodeMap<size_t> IndexMap;
			IndexMap connected_node_map(g);
			lemon::connectedComponents(g, connected_node_map);

			std::map<size_t, std::set<lemon::ListGraph::Node> > map_subgraphs;

			// Create subgraphs' map
			typedef lemon::ListGraph::NodeIt NodeIterator;
			NodeIterator itNode(g);
			for (IndexMap::MapIt it(connected_node_map);
				it != lemon::INVALID; ++it, ++itNode) {
				map_subgraphs[*it].insert(itNode);
			}
			return map_subgraphs;
		}

	} // namespace sfm
} // namespace mvg

#endif // MVG_SFM_CONNECTED_COMPONENT_H_
