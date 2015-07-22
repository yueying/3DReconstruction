/*******************************************************************************
 * 文件： scoped_lock.h
 * 时间： 2015/04/16 15:28
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： ScopedLock模板类，在作用域内对共享资源进行加锁
 *
********************************************************************************/
#ifndef FBLIB_THREADS_SCOPED_LOCK_H_
#define FBLIB_THREADS_SCOPED_LOCK_H_

namespace fblib
{
	namespace threads
	{
		template <class M> class ScopedLock
		{
		private:
			M& m_lock;
			ScopedLock(const ScopedLock&); // prevent copy
			ScopedLock& operator=(const ScopedLock&); // prevent assign
		public:
			explicit ScopedLock(M& m) :m_lock(m) { m_lock.lock(); }
			~ScopedLock(){ m_lock.unlock(); }
		};

		template <class M> class ReverseScopedLock
		{
		private:
			M& m_lock;
			ReverseScopedLock(const ReverseScopedLock&); // prevent copy
			ReverseScopedLock& operator=(const ReverseScopedLock&); // prevent assign
		public:
			explicit ReverseScopedLock(M& m) :m_lock(m) { m_lock.unlock(); }
			~ReverseScopedLock(){ m_lock.lock(); }
		};


		template <class M> class ScopedPointerLock
		{
		private:
			M* m_lock;
			ScopedPointerLock(const ScopedPointerLock&); // prevent copy
			ScopedPointerLock& operator=(const ScopedPointerLock&); // prevent assign
		public:
			explicit ScopedPointerLock(M* m) :m_lock(m) { if (m_lock) m_lock->lock(); }
			~ScopedPointerLock(){ if (m_lock) m_lock->unlock(); }
		};

		template <class M> class ReverseScopedPointerLock
		{
		private:
			M* m_lock;
			ReverseScopedPointerLock(const ReverseScopedPointerLock&); // prevent copy
			ReverseScopedPointerLock& operator=(const ReverseScopedPointerLock&); // prevent assign
		public:
			explicit ReverseScopedPointerLock(M* m) :m_lock(m) { if (m_lock) m_lock->unlock(); }
			~ReverseScopedPointerLock(){ if (m_lock) m_lock->lock(); }
		};
	}
}
#endif // FBLIB_THREADS_SCOPED_LOCK_H_
