#ifndef FBLIB_FEATURE_INDEXED_MATCH_UTILS_H_
#define FBLIB_FEATURE_INDEXED_MATCH_UTILS_H_

#include "fblib/feature/indexed_match.h"
#include <map>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace fblib {
	namespace feature {

		/**
		 * \brief	将匹配的索引值对导入到文件
		 *
		 * \param	map_indexed_matches	索引匹配对
		 * \param [in,out]	os		   	输出流
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		static bool PairedIndexedMatchToStream(
			const PairWiseMatches & map_indexed_matches,
			std::ostream & os)
		{
			for (PairWiseMatches::const_iterator iter = map_indexed_matches.begin();
				iter != map_indexed_matches.end();
				++iter)
			{
				const size_t left_index = iter->first.first;
				const size_t right_index = iter->first.second;
				const std::vector<IndexedMatch> & vec_matches = iter->second;
				os << left_index << " " << right_index << '\n' << vec_matches.size() << '\n';
				copy(vec_matches.begin(), vec_matches.end(),
					std::ostream_iterator<IndexedMatch>(os, ""));
			}
			return os.good();
		}

		/**
		 * \brief	从文件中导入匹配索引值对
		 *
		 * \param	file_name				   	要导入的文件名
		 * \param [in,out]	map_indexed_matches	匹配的索引值对
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		static bool PairedIndexedMatchImport(
			const std::string & file_name,
			PairWiseMatches & map_indexed_matches)
		{
			bool is_ok = false;
			std::ifstream in(file_name.c_str());
			if (in.is_open()) {
				map_indexed_matches.clear();

				size_t left_index, right_index, number;
				while (in >> left_index >> right_index >> number)  {
					std::vector<IndexedMatch> matches(number);
					for (size_t i = 0; i < number; ++i) {
						in >> matches[i];
					}
					map_indexed_matches[std::make_pair(left_index, right_index)] = matches;
				}
				is_ok = true;
			}
			else  {
				std::cout << std::endl << "ERROR IndexedMatchesUtils::import(...)" << std::endl
					<< "with : " << file_name << std::endl;
				is_ok = false;
			}
			return is_ok;
		}
	}  // namespace feature
}  // namespace fblib

#endif // FBLIB_FEATURE_INDEXED_MATCH_UTILS_H_
