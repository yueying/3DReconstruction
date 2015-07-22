﻿#ifndef FBLIB_FEATURE_MATCHER_H_
#define FBLIB_FEATURE_MATCHER_H_

#include "fblib/feature/indexed_match.h"
#include <map>
#include <string>
#include <vector>

namespace fblib {
	namespace feature{

		/**	图像集合匹配的接口
		 */
		class Matcher
		{
		public:
			Matcher() {};

			virtual ~Matcher() {};

			/**
			 * \brief	/ Build point indexes correspondences lists between images ids
			 * 			 the output pairwise photometric corresponding points.
			 *
			 * \param	vec_filenames				 	The vector filenames.
			 * \param [in,out]	map_putatives_matches	The map putatives matches.
			 */
			virtual void Match(
				const std::vector<std::string> & vec_filenames,
				PairWiseMatches & map_putatives_matches 
				)const = 0;
		};
	}// namespace feature
}; // namespace fblib


#endif // FBLIB_FEATURE_MATCHER_H_
