/*******************************************************************************
 * 文件： noncopyable.h
 * 时间： 2014/11/05 23:57
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 不能复制类的基类，有任何的复制操作，编译器会报错.继承这个类就不能有复制操作
 *
 * TODO: 这边下一步做一个分析
 *
********************************************************************************/
#ifndef MVG_BASE_NONCOPYABLE_H_
#define MVG_BASE_NONCOPYABLE_H_

#include <mvg/base/link_pragmas.h>

namespace mvg
{
	namespace utils
	{
		/** 不能复制类的基类，有任何的复制操作，编译器会报错.将深浅复制都写入父类的私有方法中，这样派生的子类也无法实现深浅复制
		 *  例子：
		 *
		 *  \code
		 *   class MyFancyClass : public mvg::utils::CNoncopyable
		 *   {
		 *    public:
		 *     ...
		 *   };
		 *  \endcode
		 * \ingroup mvg_base_grp
		 */
		class BASE_IMPEXP Noncopyable
		{
		protected:
			Noncopyable() {}
			~Noncopyable() {}
		private:
			Noncopyable(const Noncopyable &);  // 这不需要再其他地方实现
			Noncopyable& operator =(const Noncopyable &);   // 这不需要再其他地方实现
		}; // End of class def.

	} // End of namespace
} // end of namespace

#endif // MVG_BASE_NONCOPYABLE_H_
