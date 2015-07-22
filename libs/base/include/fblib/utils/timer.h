/*******************************************************************************
 * 文件： timer.h
 * 时间： 2014/11/09 22:15
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 这个类实现了一个高性能的码表.
 *
********************************************************************************/
#ifndef FBLIB_UTILS_TIMER_H_
#define FBLIB_UTILS_TIMER_H_

#include <fblib/base/link_pragmas.h>
#include <fblib/utils/noncopyable.h>

namespace fblib
{
namespace utils
{
	/** 这个类实现了一个高性能的秒表.
	 *  精度 1e-6 seconds.
	 * \ingroup fblib_base_grp
	 */
	class BASE_IMPEXP Timer : public fblib::utils::Noncopyable
	{
	private:
		unsigned char largeInts[64];
	public:
		/** 构造函数. */
		Timer();

		/** 析构函数. */
		virtual ~Timer();

		/** 启动码表
		 * \sa Stop
		 */
		void	Start();

		/** 停止秒表
		 * \return 返回经过的时间，单位秒
		 * \sa Start
		 */
		double	Stop();

	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif // FBLIB_UTILS_TIMER_H_
