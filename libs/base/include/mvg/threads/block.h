/*******************************************************************************
 * 文件： block.h
 * 时间： 2015/04/16 21:03
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 阻塞类，阻塞线程执行
 *
********************************************************************************/
#ifndef MVG_THREADS_BLOCK_H_
#define MVG_THREADS_BLOCK_H_

#include <mvg/threads/thread.h>
#include <mvg/threads/barrier.h>
#include <mvg/threads/condition.h>
#include <mvg/threads/scoped_lock.h>

namespace mvg 
{
	namespace threads
	{
		/** 这个类的作用是阻塞线程的执行，即 使用block()阻塞执行它的线程
		  （注意，不一定是定义它的Thread线程，而是当前执行了block()函数的线程，包括系统主线程）
		  并使用release()释放之前被阻塞的线程
		*/
		class Block
		{
		public:

			Block() : m_released(false) {}

			~Block()
			{
				release();
			}

			/**	阻塞当前线程
			 */
			inline bool block()
			{
				ScopedLock<mvg::threads::Mutex> mutlock(m_mut);
				if (!m_released)
				{
					return m_cond.wait(&m_mut) == 0;
				}
				else
				{
					return true;
				}
			}
			/**	阻塞当前线程，加入时间
			 */
			inline bool block(unsigned long timeout)
			{
				ScopedLock<mvg::threads::Mutex> mutlock(m_mut);
				if (!m_released)
				{
					return m_cond.wait(&m_mut, timeout) == 0;
				}
				else
				{
					return true;
				}
			}
			/**	释放当前的线程
			 */
			inline void release()
			{
				ScopedLock<mvg::threads::Mutex> mutlock(m_mut);
				if (!m_released)
				{
					m_released = true;
					m_cond.broadcast();
				}
			}

			inline void reset()
			{
				ScopedLock<mvg::threads::Mutex> mutlock(m_mut);
				m_released = false;
			}

			inline void set(bool doRelease)
			{
				if (doRelease != m_released)
				{
					if (doRelease) release();
					else reset();
				}
			}

		protected:

			Mutex m_mut;
			Condition m_cond;
			bool m_released;

		private:

			Block(const Block&) {}
		};

		/** 计数阻塞器类，它与阻塞器类的使用方法基本相同。使用block()阻塞线程，使用release()释放线程
		* 不过初次之外，BlockCount的构造函数还可以设置一个阻塞计数器，计数的作用是：每当阻塞器对象的
		* completed()函数被执行一次，计数器就减1，直到减到零释放被阻塞的线程
		*/
		class BlockCount
		{
		public:

			BlockCount(unsigned int blockCount) :
				m_blockCount(blockCount),
				m_currentCount(0) {}

			~BlockCount()
			{
				m_blockCount = 0;
				release();
			}
			/**	完成一次，计数器就减1，减到零的时候释放被阻塞的线程
			 */
			inline void completed()
			{
				mvg::threads::ScopedLock<mvg::threads::Mutex> mutlock(m_mut);
				if (m_currentCount > 0)
				{
					--m_currentCount;

					if (m_currentCount == 0)
					{
						m_cond.broadcast();
					}
				}
			}

			inline void block()
			{
				mvg::threads::ScopedLock<mvg::threads::Mutex> mutlock(m_mut);
				if (m_currentCount)
					m_cond.wait(&m_mut);
			}

			inline void reset()
			{
				mvg::threads::ScopedLock<mvg::threads::Mutex> mutlock(m_mut);
				if (m_currentCount != m_blockCount)
				{
					if (m_blockCount == 0) m_cond.broadcast();
					m_currentCount = m_blockCount;
				}
			}

			inline void release()
			{
				mvg::threads::ScopedLock<mvg::threads::Mutex> mutlock(m_mut);
				if (m_currentCount)
				{
					m_currentCount = 0;
					m_cond.broadcast();
				}
			}

			inline void setBlockCount(unsigned int blockCount) { m_blockCount = blockCount; }

			inline unsigned int getBlockCount() const { return m_blockCount; }

			inline unsigned int getCurrentCount() const { return m_currentCount; }

		protected:

			mvg::threads::Mutex m_mut;
			mvg::threads::Condition m_cond;
			unsigned int m_blockCount;
			unsigned int m_currentCount;

		private:

			BlockCount(const BlockCount&) {}

		};
	}
}

#endif // MVG_THREADS_BLOCK_H_
