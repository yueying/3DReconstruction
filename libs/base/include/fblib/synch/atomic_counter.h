/*******************************************************************************
 * 文件： atomic_counter.h
 * 时间： 2014/11/09 19:57
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 多线程中保证原子递增或者递减
 *
********************************************************************************/
#ifndef FBLIB_SYNCH_ATOMIC_COUNTER_H_
#define FBLIB_SYNCH_ATOMIC_COUNTER_H_

#include <fblib/base/link_pragmas.h>
#include <fblib/utils/noncopyable.h>

namespace fblib
{
	namespace synch
	{

		/** 这个类的行为和int(或者 long)类型一样,但是保证的原子的递增或者递减.
		*/
		class BASE_IMPEXP AtomicCounter:public fblib::utils::Noncopyable
		{
		public:

			typedef long atomic_num_t;

			explicit AtomicCounter(long v) : m_value(static_cast<atomic_num_t>(v))
			{ }

			void operator++();  //!< 原子增量的值.
			atomic_num_t operator--(); 	//!< 原子递减，返回一个新值.
			operator atomic_num_t() const; //!< 得到当前值

		private:
			mutable atomic_num_t m_value;

		}; // end of AtomicCounter

	} // End of namespace
} // End of namespace

#endif // FBLIB_SYNCH_ATOMIC_COUNTER_H_