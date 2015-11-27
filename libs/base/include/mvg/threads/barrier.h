/*******************************************************************************
 * 文件： barrier.h
 * 时间： 2015/04/14 19:28
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 线程栅栏类
 *
********************************************************************************/
#ifndef MVG_THREADS_BARRIER_H_
#define MVG_THREADS_BARRIER_H_

#include <mvg/base/link_pragmas.h>

namespace mvg 
{
	namespace threads
	{

		/**
		  线程栅栏类，这是一个对线程同步来说比较重要的阻塞器接口，其构造函数与BlockCount类似，可以设置一个整数值，
		  我们可以把这个理解为栅栏的“强度”.每个执行了Barrier::block()函数的线程都将被阻塞，当被阻塞在栅栏处的线程达到指定
		  的数目时，栅栏就会被冲开，所有的线程将被释放。重要的是，这些线程是几乎同时释放，也就保证了线程执行的同步性
		 */
		class BASE_IMPEXP Barrier {

		public:

			/**
			 *  构造函数
			 */
			Barrier(int numThreads = 0);

			/**
			 *  析构函数
			 */
			virtual ~Barrier();

			/**
			 *  将barrier重置到原始状态
			 */
			virtual void reset();

			/**
			 *  阻塞当前的线程，如果超过栅栏的强度（可以重新设定强度），则自动释放所有的线程
			 */
			virtual void block(unsigned int numThreads = 0);

			/**
			 *  释放barrier
			 */
			virtual void release();

			/**
			 *  返回当前的barrier阻塞的线程数，
			 *  -1表示有误
			 */
			virtual int numThreadsCurrentlyBlocked();


			void invalidate();

		private:

			/**
			 *  复制构造函数私有化，防止串改
			 */
			Barrier(const Barrier &/*b*/) {};

			/**
			 *  赋值操作私有化，防止串改
			 */
			Barrier &operator=(const Barrier &/*b*/) { return *(this); };

			/**
			 *  私有成员数据
			 */
			void *m_prvData;


			bool _valid;

		};

	}
}

#endif // MVG_THREADS_BARRIER_H_

