#ifndef FBLIB_FEATURE_INDEXED_MATCH_H_
#define FBLIB_FEATURE_INDEXED_MATCH_H_

#include <iostream>
#include <set>
#include <map>
#include <vector>

namespace fblib {
	namespace feature {

		/**
		 * \brief	用于保存成对的索引，并对索引进行排序，删除重复的匹配对
		 */		
		struct IndexedMatch
		{
			IndexedMatch(size_t i = 0, size_t j = 0)  {
				_i = i;
				_j = j;
			}

			friend bool operator==(const IndexedMatch &m1, const IndexedMatch &m2)  {
				return (m1._i == m2._i && m1._j == m2._j);
			}

			friend bool operator!=(const IndexedMatch &m1, const IndexedMatch &m2)  {
				return !(m1 == m2);
			}


			/**
			 * \brief	按字典顺序进行排序，删除重复值
			 *
			 * \param	m1	第一个匹配项
			 * \param	m2	第二个匹配项
			 *
			 * \return	true if the first parameter is less than the second.
			 */
			friend bool operator<(const IndexedMatch& m1, const IndexedMatch& m2) {
				if (m1._i < m2._i)
					return m1._j < m2._j;
				else if (m1._i > m2._i)
					return m1._j < m2._j;
				return m1._i < m2._i;
			}

			/**	删除重复的项（相同的_i或_j出现多次）
			 */
			static bool getDeduplicated(std::vector<IndexedMatch> &vec_match){
				size_t size_before = vec_match.size();
				//使用set，set会去除相同项
				std::set<IndexedMatch> set_deduplicated(vec_match.begin(), vec_match.end());
				vec_match.assign(set_deduplicated.begin(), set_deduplicated.end());
				return size_before != vec_match.size();
			}

			size_t _i, _j;  //!< 左右索引
		};

		static std::ostream& operator<<(std::ostream &out, const IndexedMatch &obj) {
			return out << obj._i << " " << obj._j << std::endl;
		}

		static inline std::istream& operator>>(std::istream &in, IndexedMatch &obj) {
			return in >> obj._i >> obj._j;
		}

		// 对图像集匹配进行构建索引，索引项为pair<i,j>，值为图像i和图像j的匹配对 
		typedef std::map< std::pair<size_t, size_t>, std::vector<feature::IndexedMatch> > PairWiseMatches;

	}  // namespace feature
}  // namespace fblib

#endif // FBLIB_FEATURE_INDEXED_MATCH_H_

