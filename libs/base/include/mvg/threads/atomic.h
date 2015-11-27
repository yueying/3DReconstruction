/*******************************************************************************
 * 文件： atomic.h
 * 时间： 2015/04/14 19:27
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 原子操作
 *
********************************************************************************/
#ifndef MVG_THREADS_ATOMIC_H_
#define MVG_THREADS_ATOMIC_H_

#include <mvg/base/link_pragmas.h>

namespace mvg {

	namespace threads{
		/**
		 *  用于提供原子的递增递减操作
		 */
		class BASE_IMPEXP Atomic {
		public:
			Atomic(unsigned value = 0) : m_value(value)
			{ }
			unsigned operator++();
			unsigned operator--();
			unsigned AND(unsigned value);
			unsigned OR(unsigned value);
			unsigned XOR(unsigned value);
			unsigned exchange(unsigned value = 0);
			operator unsigned() const;
		private:

			Atomic(const Atomic&);
			Atomic& operator=(const Atomic&);

			volatile long m_value;

		};

		/**
		 *  提供原子指针操作
		 */
		class BASE_IMPEXP AtomicPtr {
		public:
			AtomicPtr(void* ptr = 0) : m_ptr(ptr)
			{ }
			~AtomicPtr()
			{
				m_ptr = 0;
			}

			// 分配一个新的指针
			bool assign(void* ptrNew, const void* const ptrOld);
			void* get() const;

		private:
			AtomicPtr(const AtomicPtr&);
			AtomicPtr& operator=(const AtomicPtr&);

			void* volatile m_ptr;
		};

	}
}

#endif // MVG_THREADS_ATOMIC_H_
