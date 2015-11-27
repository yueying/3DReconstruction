#ifndef MVG_UTILS_SVG_DRAWER_H_
#define MVG_UTILS_SVG_DRAWER_H_

#include <sstream>
#include <fstream>
#include <vector>

namespace mvg {
	namespace utils{

		
		/** \brief	基本的svg格式 */
		class SvgStyle
		{		
		public:

			SvgStyle() :fill_color_(""), stroke_color_("black"), tool_tip_(""),
				stroke_width_(1.0f) {}

			/**配置填充的颜色
			 */
			SvgStyle & fill(const std::string &fill_color)
			{
				fill_color_ = fill_color; 
				return *this;
			}

			/**	配置描边的颜色和宽度
			 */
			SvgStyle & stroke(const std::string & stroke_color, float stroke_width = 1.f)
			{
				stroke_color_ = stroke_color;  
				stroke_width_ = stroke_width; 
				return *this;
			}

			/**	设置不进行描边
			 */
			SvgStyle & noStroke()
			{
				stroke_color_ = "";  
				stroke_width_ = 0.f; 
				return *this;
			}

			/**	配置提示信息
			 */
			SvgStyle & tooltip(const std::string & tool_tip)
			{
				tool_tip_ = tool_tip; return *this;
			}
			/**	得到svg流信息
			 */
			const std::string getSvgStream() const {
				std::ostringstream os;

				if (!stroke_color_.empty())
					os << " stroke=\"" << stroke_color_ << "\" stroke-width=\"" << stroke_width_ << "\"";
				if (fill_color_.empty())
					os << " fill=\"none\"";
				else
					os << " fill=\"" << fill_color_ << "\"";
				if (!tool_tip_.empty())
					os << " tooltip=\"enable\">"
					<< "<title>" << tool_tip_ << "</title>";

				return os.str();
			}

			bool isTooltip() const { 
				return !tool_tip_.empty(); 
			}
		private:
			std::string fill_color_, stroke_color_, tool_tip_;
			float stroke_width_;
		};

		/**
		 * \brief	简单的svg绘制处理类，可以绘制直线，正方形，矩阵，文本以及链接图像
		 */
		class SvgDrawer{
		public:
			
			SvgDrawer(size_t width = 0, size_t height = 0)
			{
				svg_stream_ << "<?xml version=\"1.0\" standalone=\"yes\"?>\n";

				svg_stream_ << "<!-- SVG graphic -->" << std::endl
					<< "<svg xmlns='http://www.w3.org/2000/svg'"
					<< " xmlns:xlink='http://www.w3.org/1999/xlink'" << "\n";

				if (width > 0 && height > 0)
					svg_stream_ << "width=\"" << width << "px\" height=\"" << height << "px\""
					<< " preserveAspectRatio=\"xMinYMin meet\""
					<< " viewBox=\"0 0 " << width << ' ' << height << "\"";

				svg_stream_ << " version=\"1.1\">" << std::endl;
			}

			/**	绘制圆，中心点的坐标和半径
			 */
			void drawCircle(float cx, float cy, float r,
				const SvgStyle & style)
			{
				svg_stream_ << "<circle cx=\"" << cx << "\"" << " cy=\"" << cy << "\""
					<< " r=\"" << r << "\""
					<< style.getSvgStream() + (style.isTooltip() ? "</circle>" : "/>\n");
			}

			/**	绘制直线，起始点和结束点
			 */
			void drawLine(float ax, float ay, float bx, float by,
				const SvgStyle &style)
			{
				svg_stream_ <<
					"<polyline points=\"" << ax << "," << ay << "," << bx << "," << by << "\""
					<< style.getSvgStream() + (style.isTooltip() ? "</polyline>" : "/>\n");
			}

			/**	绘制图像，给出图像的链接，路径必须是相对于svg文件
			 */
			void drawImage(const std::string & image_path, int width, int height,
				int posx = 0, int posy = 0, float opacity = 1.f)
			{
				svg_stream_ <<
					"<image x=\"" << posx << "\"" << " y=\"" << posy << "\""
					<< " width=\"" << width << "px\"" << " height=\"" << height << "px\""
					<< " opacity=\"" << opacity << "\""
					<< " xlink:href=\"" << image_path << "\"" << "/>\n";
			}

			/**	绘制正方形，x，y位置和正方形边长
			 */
			void drawSquare(float cx, float cy, float width,
				const SvgStyle & style)
			{
				drawRectangle(cx, cy, width, width, style);
			}

			/**	绘制矩形，x,y位置和长方形的宽和高
			 */
			void drawRectangle(float cx, float cy, float width, float height,
				const SvgStyle & style)
			{
				svg_stream_ << "<rect x=\"" << cx << "\""
					<< " y=\"" << cy << "\""
					<< " width=\"" << width << "\""
					<< " height=\"" << height << "\""
					<< style.getSvgStream() + (style.isTooltip() ? "</rect>" : "/>\n");
			}

			/**	显示文本，给出x,y的位置以及字体的大小
			 */
			void drawText(float cx, float cy, float font_size = 1.0f, const std::string & stext = "",
				const std::string & scol = "")
			{
				svg_stream_ << "<text" << " x=\"" << cx << "\"" << " y=\"" << cy << "\""
					<< " font-size=\"" << font_size << "\"";
				if (!scol.empty())
					svg_stream_ << " fill=\"" << scol << "\"";

				svg_stream_ << ">" << stext << "</text>\n";
			}

			template< typename DataInputIteratorX, typename DataInputIteratorY>
			void drawPolyline(DataInputIteratorX x_start, DataInputIteratorX x_end,
				DataInputIteratorY y_start, DataInputIteratorY y_end,
				const SvgStyle &style)
			{
				svg_stream_ << "<polyline points=\"";

				DataInputIteratorY itery = y_start;
				for (DataInputIteratorX iterx = x_start;
					iterx != x_end; std::advance(iterx, 1), std::advance(itery, 1))
				{
					svg_stream_ << *iterx << ',' << *itery << ' ';
				}
				svg_stream_ << "\""
					<< style.getSvgStream() + (style.isTooltip() ? "</polyline>" : "/>\n");
			}

			/**	svg的结束标志
			 */
			std::ostringstream & closeSvgFile()
			{
				svg_stream_ << "</svg>";
				return svg_stream_;
			}
		private:
			std::ostringstream svg_stream_;
		};

		
		
		/** \brief	用于绘制svg直方图  
		*  ____
		*  |  |   ___ |
		*  |  |__|  | |
		*  |  |  |  | |
		*  -----------|
		*/
		struct SvgHistogram
		{
			template<typename T>
			static std::string stringifier(const T & t)
			{
				std::ostringstream os;
				os << t;
				return os.str();
			}

			template<typename T>
			void draw(const std::vector<T> &vec_value,
				const std::pair<float, float> &range,
				const std::string &file_name,
				const float width, const float height)
			{
				if (vec_value.empty())  {
					return;
				}
				// 给出最大值
				T maxi = *std::max_element(vec_value.begin(), vec_value.end());
				size_t n = vec_value.size();

				float scale_factor_y = height / static_cast<float>(maxi);
				float scale_factor_x = width / static_cast<float>(n);

				SvgDrawer svg_stream_;

				for (typename std::vector<T>::const_iterator iter = vec_value.begin();
					iter != vec_value.end();
					++iter)
				{
					size_t dist = std::distance(vec_value.begin(), iter);
					T val = *iter;
					std::ostringstream os;
					os << '(' << range.first + dist / float(n) * (range.second - range.first) << ',' << val << ')';
					SvgStyle style = SvgStyle().fill("blue").stroke("black", 1.0).tooltip(os.str());
					svg_stream_.drawRectangle(
						scale_factor_x * dist, height - val * scale_factor_y,
						scale_factor_x, val * scale_factor_y,
						style);
					//_________
					//|       |_________
					//|       ||       |
					//|       ||       |
					//|       ||       |
					//0    factor_x  2*factor_x
				}
				SvgStyle style_axis = SvgStyle().stroke("black", 1.0f);
				// 画出X轴
				svg_stream_.drawText(.05f*width, 1.2f*height, .1f*height, stringifier(range.first), "black");
				svg_stream_.drawText(width, 1.2f*height, .1f*height, stringifier(range.second), "black");
				svg_stream_.drawLine(0, 1.1f*height, width, 1.1f*height, style_axis);
				// 画出Y轴
				svg_stream_.drawText(1.2f*width, .1f*height, .1f*height, stringifier(maxi), "black");
				svg_stream_.drawText(1.2f*width, height, .1f*height, "0", "black");
				svg_stream_.drawLine(1.1f*width, 0, 1.1f*width, height, style_axis);

				std::ofstream svg_file_stream(file_name.c_str());
				svg_file_stream << svg_stream_.closeSvgFile().str();
			}
		};
	}// namespace utils
} // namespace mvg

#endif // MVG_UTILS_SVG_DRAWER_H_
