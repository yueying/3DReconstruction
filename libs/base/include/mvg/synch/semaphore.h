/*******************************************************************************
 * 文件： Semaphore.h
 * 时间： 2014/11/13 12:59
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 信号量设置
 *
********************************************************************************/
#ifndef MVG_SYNCH_SEMAPHORE_H_
#define MVG_SYNCH_SEMAPHORE_H_

#include <mvg/base/link_pragmas.h>
#include <string>
#include <mvg/utils/referenced_memory_block.h>

namespace mvg
{
	namespace synch
	{
		/** 线程间的信号同步.
		  * 信号量在计数大于0时表示触发状态,调用waitForSignal不会阻塞，等于0表示未触发状态
		  * 调用waitForSignal会阻塞直到有其它线程递增了计数。
		  * 在对信号量调用等待函数时，等待函数会检查信号量的当前资源计数，
		  * 如果大于0(即信号量处于触发状态)，减1后返回让调用线程继续执行。一个线程可以多次调用等待函数来减小信号量。 
		  * initialCount参数表示初始资源数量
		  * maxCount参数表示最大并发数量
		  * name参数表示信号量的名称，空表示匿名信号量
		  */
		class BASE_IMPEXP Semaphore
		{
		protected:
			mvg::utils::ReferencedMemoryBlock m_data;
			std::string  m_name; //!< 信号量的名称，空表示匿名信号量.

        public:
            /** 创建一个信号量
              * 如果\a name不是一个空的字符串,带名称的信号量被创建.在这种情况下，如果信号量不存在则创建
              * 否则直接将存在的信号量直接链接到这个对象，忽略参数\a initialCount和\a maxCount.
              */
            Semaphore(
                unsigned int    initialCount,
                unsigned int    maxCount,
                const std::string &name=std::string("") );

            /** 析构函数
              */
            virtual ~Semaphore();

			/** 阻塞，直到信号量非0
			  *\param timeout_ms 设置超时时间（单位毫秒）,或者设置为0无限等待.
			  *\return true信号量设置成功, false表示超时或者其它错误.
              */
            bool waitForSignal( unsigned int timeout_ms = 0 );

            /** 通过给定数目递增信号量的当前资源计数
              */
            void release(unsigned int increaseCount = 1);

			/**得到一个命名的信息量的名字，如果信息量是匿名的则返回空字符串 */
			inline std::string  getName() const { return m_name; }

			/** 返回true如果这个信号量是命名的 */
			inline bool isNamed() const { return !m_name.empty(); }


		};

	} // End of namespace

} // End of namespace

#endif // MVG_SYNCH_SEMAPHORE_H_
