/*******************************************************************************
 * 文件： mutex.h
 * 时间： 2015/04/16 19:19
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 互斥体接口类
 *
********************************************************************************/
#ifndef MVG_THREADS_MUTEX_H_
#define MVG_THREADS_MUTEX_H_

#include <mvg/base/link_pragmas.h>

namespace mvg 
{
	namespace threads
	{
		/**
		 *  提供一个面向对象的线程互斥体接口类
		 */
		class BASE_IMPEXP Mutex 
		{
			friend class Condition;

		public:

			enum MutexType
			{
				MUTEX_NORMAL,
				MUTEX_RECURSIVE
			};

			/**
			 *  构造函数
			 */
			Mutex(MutexType type = MUTEX_NORMAL);

			/**
			 *  析构函数
			 */
			virtual ~Mutex();


			MutexType getMutexType() const { return m_mutexType; }


			/**
			 *  互斥体加锁函数
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			virtual int lock();

			/**
			 *  互斥体解锁函数
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			virtual int unlock();

			/**
			 *  测试互斥体可以被加锁
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			virtual int trylock();

		private:

			/**
			 *  复制构造函数私有化，防止串改
			 */
			Mutex(const Mutex &/*m*/) {};

			/**
			 *  赋值操作私有化，防止串改
			 */
			Mutex &operator=(const Mutex &/*m*/) { return *(this); };

			/**
			 *  私有成员
			 */
			void *m_prvData;
			MutexType m_mutexType;

		};
	}
}

#endif // MVG_THREADS_MUTEX_H_
