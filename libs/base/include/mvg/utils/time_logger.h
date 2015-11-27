/*******************************************************************************
 * 文件： time_logger.h
 * 时间： 2014/12/02 10:22
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 构建一个多用途的时间分析器
 *
********************************************************************************/
#ifndef MVG_UTILS_TIME_LOGGER_H_
#define MVG_UTILS_TIME_LOGGER_H_

#include <mvg/utils/mvg_macros.h>

#include <vector>
#include <stack>
#include <map>

#include <mvg/utils/timer.h>

namespace mvg
{
	namespace utils
	{
		/**	一个通用的分析器，对每个enter(X)-leave(X)的时间进行记录.
		 */
		class BASE_IMPEXP TimeLogger
		{
		private:
			Timer m_stopwatch;//码表
			bool m_enabled;//定义一个全局变量，确定是否启用时间分析器

			//! 最终显示所有调用的数据:
			struct BASE_IMPEXP TCallData
			{
				TCallData();

				size_t n_calls;//调用次数
				double min_t, max_t, mean_t;//调用的最小，最大，平均时间
				std::stack<double, std::vector<double> >   open_calls;
				bool has_time_units;
			};

			std::map<std::string, TCallData>  m_data;

			void do_enter(const char *func_name);
			double do_leave(const char *func_name);

		public:
			/**	每个时间分析段信息
			 */
			struct BASE_IMPEXP TCallStats
			{
				size_t n_calls;//调用次数
				double min_t, max_t, mean_t, total_t;//最小，最大，平均，和总时间
			};
			TimeLogger(bool enabled = true); //! 默认构造函数，默认启动
			virtual ~TimeLogger(); //!< 析构函数
		};
	}
}

#endif // MVG_UTILS_TIME_LOGGER_H_