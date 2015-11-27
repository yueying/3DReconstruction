#ifndef MVG_SFM_TRIPLET_FINDER_H
#define MVG_SFM_TRIPLET_FINDER_H

#include "lemon/list_graph.h"
using namespace lemon;

#include <algorithm>
#include <vector>

namespace mvg{
	namespace sfm{

		
		/** \brief	简单的存放三元组的容器，主要用来存储图节点的id */
		struct Triplet
		{
			Triplet(size_t ii, size_t jj, size_t kk)
				:i(ii), j(jj), k(kk)
			{ }
			size_t i, j, k;

			bool contain(const std::pair<size_t, size_t> & edge) const
			{
				size_t It = edge.first;
				size_t Jt = edge.second;
				if ((It == i || It == j || It == k) &&
					(Jt == i || Jt == j || Jt == k) && It != Jt)
					return true;
				else
					return false;
			}

			friend bool operator==(const Triplet& m1, const Triplet& m2)  {
				return m1.contain(std::make_pair(m2.i, m2.j))
					&& m1.contain(std::make_pair(m2.i, m2.k));
			}

			friend bool operator!=(const Triplet& m1, const Triplet& m2)  {
				return !(m1 == m2);
			}

			friend std::ostream & operator<<(std::ostream & os, const Triplet & t)
			{
				os << t.i << " " << t.j << " " << t.k << std::endl;
				return os;
			}
		};

		/**
		 * \brief	用于返回图中的所有三元组，采用广度优先遍历
		 *
		 * \tparam	GraphT	图的类型
		 * \param	g						待处理的图
		 * \param [in,out]	vec_triplets	存放三元组的向量，传入必须为空值
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		template<typename GraphT>
		bool ListTriplets(const GraphT &g, std::vector< Triplet > &vec_triplets)
		{
			// 具体算法： 访问与节点连接没有访问过的边，对这些边进行遍历，如果边的节点相连，
			// 则检测这个圆的长度是否为3。
			typedef typename GraphT::OutArcIt OutArcIt;
			typedef typename GraphT::NodeIt NodeIterator;
			typedef typename GraphT::template EdgeMap<bool> BoolEdgeMap;

			BoolEdgeMap map_edge(g, false); // 确定边是否已经访问

			// 对每个节点进行处理
			for (NodeIterator iter_node(g); iter_node != INVALID; ++iter_node)
			{
				// 对每条边处理列出对外没有访问的边
				std::vector<OutArcIt> vec_edges;
				for (OutArcIt e(g, iter_node); e != INVALID; ++e)
				{
					if (!map_edge[e]) //如果没有访问，则添加
						vec_edges.push_back(e);
				}

				// 对所有链接的边进行遍历
				while (vec_edges.size() > 1)
				{
					OutArcIt it_prev = vec_edges[0]; 
					for (size_t i = 1; i < vec_edges.size(); ++i)
					{
						// 检测两条边的末端是否链接
						typename GraphT::Arc cycle_edge = findArc(g, g.target(it_prev), g.target(vec_edges[i]));
						if (cycle_edge != INVALID && !map_edge[cycle_edge])
						{
							// 找到基础的圆
							int triplet[3] = {
								g.id(iter_node),
								g.id(g.target(it_prev)),
								g.id(g.target(vec_edges[i])) };
							std::sort(&triplet[0], &triplet[3]);
							vec_triplets.push_back(Triplet(triplet[0], triplet[1], triplet[2]));
						}
					}
					// 标识当前边已被访问
					map_edge[it_prev] = true;
					// 移除第一条边
					vec_edges.erase(vec_edges.begin());
				}
			}
			return (!vec_triplets.empty());
		}

	} // namespace sfm
} // namespace mvg

#endif // MVG_SFM_TRIPLET_FINDER_H
