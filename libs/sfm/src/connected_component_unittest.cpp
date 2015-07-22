#include <iostream>
#include <vector>

#include "testing.h"

#include "fblib/sfm/connected_component.h"
#include "lemon/list_graph.h"
using namespace lemon;

TEST(ConnectedComponents, Empty) {
  lemon::ListGraph graph;

  int connected_component_count = lemon::countConnectedComponents(graph);
  EXPECT_EQ(0, connected_component_count);
}

TEST(ConnectedComponents, OneCC) {
  lemon::ListGraph graph;
  lemon::ListGraph::Node a = graph.addNode(), b = graph.addNode();
  graph.addEdge(a,b);
  int connected_component_count = lemon::countConnectedComponents(graph);
  EXPECT_EQ(1, connected_component_count);
}

TEST(ConnectedComponents, TwoCC) {
  lemon::ListGraph graph;

  lemon::ListGraph::Node a = graph.addNode(), b = graph.addNode();
  graph.addEdge(a,b);
  lemon::ListGraph::Node a2 = graph.addNode(), b2 = graph.addNode();
  graph.addEdge(a2,b2);
  int connected_component_count = lemon::countConnectedComponents(graph);
  EXPECT_EQ(2, connected_component_count);
}


TEST(ConnectedComponents, TwoCC_Parsing) {
  lemon::ListGraph graph;

  lemon::ListGraph::Node a = graph.addNode(), b = graph.addNode();
  graph.addEdge(a,b);
  lemon::ListGraph::Node a2 = graph.addNode(), b2 = graph.addNode();
  graph.addEdge(a2,b2);

  typedef ListGraph::NodeMap<size_t> IndexMap;
  IndexMap connected_node_map(graph);
  int connected_component_count =  lemon::connectedComponents(graph, connected_node_map);
  EXPECT_EQ(2, connected_component_count);
  for (IndexMap::MapIt it(connected_node_map); it != INVALID; ++it)
  {
    std::cout << *it << "\t" << graph.id(it) << "\n";
  }
}


/// 测试通过返回每个连接组件的节点数
// a
//
// b-c
//
// d-g
// | |
// e-f
//
// h-i-j-k
//   |/
//   l
TEST(ExportGraphToMapSubgraphs, CC_Subgraph) {
  lemon::ListGraph graph;

  // single
  lemon::ListGraph::Node a = graph.addNode();

  // two
  lemon::ListGraph::Node b = graph.addNode(), c = graph.addNode();
  graph.addEdge(b,c);

  // four
  lemon::ListGraph::Node d = graph.addNode(), e = graph.addNode(),
    f = graph.addNode(), g = graph.addNode();
  graph.addEdge(d,e);
  graph.addEdge(e,f);
  graph.addEdge(f,g);
  graph.addEdge(g,d);

  // five
  lemon::ListGraph::Node h = graph.addNode(), i = graph.addNode(),
    j = graph.addNode(), k = graph.addNode(),l = graph.addNode();
  graph.addEdge(h,i);
  graph.addEdge(i,j);
  graph.addEdge(j,k);
  graph.addEdge(i,l);
  graph.addEdge(j,l);

  const std::map<size_t, std::set<lemon::ListGraph::Node> > map_subgraphs =
    fblib::sfm::ExportGraphToMapSubgraphs(graph);

  EXPECT_EQ(4, map_subgraphs.size());
  EXPECT_EQ(5, map_subgraphs.at(0).size());
  EXPECT_EQ(4, map_subgraphs.at(1).size());
  EXPECT_EQ(2, map_subgraphs.at(2).size());
  EXPECT_EQ(1, map_subgraphs.at(3).size());
}

