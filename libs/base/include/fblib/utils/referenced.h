/*******************************************************************************
 * 文件： referenced.h
 * 时间： 2015/04/15 19:29
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 提供引用计数对象的基类
 *
********************************************************************************/
#ifndef FBLIB_UTILS_REFERENCED_H_
#define FBLIB_UTILS_REFERENCED_H_

#include <fblib/base/link_pragmas.h>

#include <fblib/threads/scoped_lock.h>
#include <fblib/threads/mutex.h>
#include <fblib/threads/atomic.h>

namespace fblib
{
	namespace utils{
		// 前置申明
		class DeleteHandler;

		/** 模板类用于帮助执行静态初始化命令 */
		template <typename T, T M()>
		struct depends_on
		{
			depends_on() { M(); }
		};

		/** 提供引用计数对象的基类 */
		class BASE_IMPEXP Referenced
		{
		public:

			Referenced();

			explicit Referenced(bool threadSafeRefUnref);

			Referenced(const Referenced&);

			inline Referenced& operator = (const Referenced&) { return *this; }

			/** 设置是否使用一个mutex来确保ref()和unref()线程安全 */
			virtual void setThreadSafeRefUnref(bool threadSafe);

			/** 得到是否一个mutex用来确保ref()和unref()线程安全*/
			bool getThreadSafeRefUnref() const { return true; }

			/** 得到用于确保ref()/unref()线程安全的mutex. */
			fblib::threads::Mutex* getRefMutex() const { return getGlobalReferencedMutex(); }

			/** 得到可选的全局 Referenced mutex, 这个可以被使用到所有的fblib::utils::Referenced中*/
			static fblib::threads::Mutex* getGlobalReferencedMutex();

			/** 引用计数递增1，表明这个对象有另外一个引用它的指针 */
			inline int ref() const;

			/** 引用计算减1，表明指向此对象的指针不在引用它，如果引用计数为0，它假定此
			 对象不再被引用，会自动删除*/
			inline int unref() const;

			/** 引用计算减1，表明指向此对象的指针不在引用它，然后不会删除它，即使ref计数为0
			警告： unref_nodelete() 只有用户可以准确的知道谁负责这个对象
				一般首选unref()，因为unref_nodelete()可能会导致内存泄露 */
			int unref_nodelete() const;

			/** 返回当前引用此对象的指针数 */
			inline int referenceCount() const { return m_refCount; }

		public:

			friend class DeleteHandler;

			/** 设置一个DeleteHandler用于委派删除所有的引用计数对象*/
			static void setDeleteHandler(DeleteHandler* handler);

			/** 得到一个DeleteHandler.*/
			static DeleteHandler* getDeleteHandler();

		protected:

			virtual ~Referenced();

			void deleteUsingDeleteHandler() const;

			mutable fblib::threads::Atomic     m_refCount;

		};

		inline int Referenced::ref() const
		{
			return ++m_refCount;
		}

		inline int Referenced::unref() const
		{
			int newRef;
			newRef = --m_refCount;
			bool needDelete = (newRef == 0);
			return newRef;
		}

		// intrusive_ptr_add_ref and intrusive_ptr_release allow
		// 提供boost::intrusive_ptr 一种“侵入式”的引用计数指针
		inline void intrusive_ptr_add_ref(Referenced* p) { p->ref(); }
		inline void intrusive_ptr_release(Referenced* p) { p->unref(); }
	}
}

#endif // FBLIB_UTILS_REFERENCED_H_
