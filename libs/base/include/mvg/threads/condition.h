/*******************************************************************************
 * 文件： condition.h
 * 时间： 2015/04/14 19:30
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 条件量接口类，它依赖于某个Mutex互斥体，互斥体加锁时阻塞所在线程，解锁
 *        或者超时则释放此线程，允许其继续运行
 *
********************************************************************************/
#ifndef MVG_THREADS_CONDITION_H_
#define MVG_THREADS_CONDITION_H_

#include <mvg/base/link_pragmas.h>
#include <mvg/threads/mutex.h>

namespace mvg 
{
	namespace threads
	{
		/**
		 *  条件量接口类
		 */
		class BASE_IMPEXP Condition {

		public:

			/**
			 *  构造函数
			 */
			Condition();

			/**
			 *  析构函数
			 */
			virtual ~Condition();

			/**
			 *  设置作为条件量的互斥体，并强制线程等待此条件满足
			 */
			virtual int wait(Mutex *mutex);

			/**
			 *  设置作为条件量的互斥体，并强制线程等待此条件满足，并添加等待时间限制
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			virtual int wait(Mutex *mutex, unsigned long int ms);

			/**
			 *  唤醒一个线程
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			virtual int signal();

			/**
			 *  唤醒所有被阻塞的线程
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			virtual int broadcast();

		private:

			/**
			 *  复制构造函数私有化，防止串改
			 */
			Condition(const Condition &/*c*/) {};

			/**
			 *  赋值操作私有化，防止串改
			 */
			Condition &operator=(const Condition &/*c*/) { return *(this); };

			/**
			 *  私有成员数据
			 */
			void *m_prvData;

		};
	}
}

#endif // MVG_THREADS_CONDITION_H_
