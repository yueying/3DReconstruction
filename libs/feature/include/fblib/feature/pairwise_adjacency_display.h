#ifndef FBLIB_FEATURE_PAIRWISE_ADJACENCY_DISPLAY_H_
#define FBLIB_FEATURE_PAIRWISE_ADJACENCY_DISPLAY_H_

#include "fblib/utils/svg_drawer.h"
#include "fblib/feature/indexed_match.h"
using namespace fblib::feature;
using namespace fblib::utils;

namespace fblib  {
	namespace feature{
		
		/**	将匹配图像通过邻接矩阵的方式进行存储，存储为svg格式
		 */
		void PairWiseMatchingToAdjacencyMatrixSVG(const size_t image_nums,
			const PairWiseMatches &map_matches,
			const std::string &output_name)
		{
			if (!map_matches.empty())
			{
				float scale_factor = 5.0f;
				SvgDrawer svg_stream((image_nums + 3) * 5, (image_nums + 3) * 5);
				// 对所有可能的匹配进行遍历
				for (size_t I = 0; I < image_nums; ++I) {
					for (size_t J = 0; J < image_nums; ++J) {
						// 如果这对图像匹配，则在I,J的位置上画上蓝色盒子
						PairWiseMatches::const_iterator iter_search =
							map_matches.find(std::make_pair(I, J));
						if (iter_search != map_matches.end() && !iter_search->second.empty())
						{
							// Display as a tooltip: (IndexI, IndexJ NbMatches)
							std::ostringstream os;
							os << "(" << J << "," << I << " " << iter_search->second.size() << ")";
							svg_stream.drawSquare(J*scale_factor, I*scale_factor, scale_factor / 2.0f,
								SvgStyle().fill("blue").noStroke());
						} // HINT : THINK ABOUT OPACITY [0.4 -> 1.0] TO EXPRESS MATCH COUNT
					}
				}
				// Display axes with 0 -> image_nums annotation : _|
				std::ostringstream os_image_nums;   os_image_nums << image_nums;
				svg_stream.drawText((image_nums + 1)*scale_factor, scale_factor, scale_factor, "0", "black");
				svg_stream.drawText((image_nums + 1)*scale_factor,
					(image_nums)*scale_factor - scale_factor, scale_factor, os_image_nums.str(), "black");
				svg_stream.drawLine((image_nums + 1)*scale_factor, 2 * scale_factor,
					(image_nums + 1)*scale_factor, (image_nums)*scale_factor - 2 * scale_factor,
					SvgStyle().stroke("black", 1.0));

				svg_stream.drawText(scale_factor, (image_nums + 1)*scale_factor, scale_factor, "0", "black");
				svg_stream.drawText((image_nums)*scale_factor - scale_factor,
					(image_nums + 1)*scale_factor, scale_factor, os_image_nums.str(), "black");
				svg_stream.drawLine(2 * scale_factor, (image_nums + 1)*scale_factor,
					(image_nums)*scale_factor - 2 * scale_factor, (image_nums + 1)*scale_factor,
					SvgStyle().stroke("black", 1.0));

				std::ofstream svg_file_stream(output_name.c_str());
				svg_file_stream << svg_stream.closeSvgFile().str();
			}
		}
	}// namespace feature
} // namespace fblib

#endif // FBLIB_FEATURE_PAIRWISE_ADJACENCY_DISPLAY_H_
