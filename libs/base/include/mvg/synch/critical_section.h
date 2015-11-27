/*******************************************************************************
 * 文件： CriticalSection.h
 * 时间： 2014/11/13 13:36
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 多线程中关键段（临界区）的封装
 *
********************************************************************************/
#ifndef MVG_SYNCH_CRITICAL_SECTION_H_
#define MVG_SYNCH_CRITICAL_SECTION_H_

#include <mvg/base/link_pragmas.h>
#include <mvg/utils/referenced_memory_block.h>
#include <string>

namespace mvg
{
	/** 这个命名空间提供了多任务，同步的工具
	 */
	namespace synch
	{
		/** 这个类提供简单的临界区的功能.
		  */
		class BASE_IMPEXP CriticalSection
		{

		private:
			mvg::utils::ReferencedMemoryBlock m_data; 
			std::string		m_name;

		public:
			/** 构造函数，初始化
			  */
			CriticalSection(const char *name = NULL);

			/** 析构函数
			  */
			~CriticalSection();

			/** 进入关键区域，系统保证各个线程互斥的进入关键区域.
			  * \exception 如果调用的线程已经进入关键区 (这个可能会造成死锁).
			  */
			void  enter() const;

			/** 离开关键区域
			  * \exception 如果调用的线程不是当前关键区的所有者.
			  */
			void  leave() const;

			/** 返回在构造函数中使用的名称 */
			std::string getName() const { return m_name; }

		};

		/** 一个类进入临界区在构造函数中，离开在析构函数中.
		  * 对于临界区问题可以多使用CriticalSectionLocker,因为这个可能是更安全的，在有些问题中程序异常结束，它仍可以释放在析构函数中.
		  *  例:
		  *  \code
		  *		{  // 代码在这个范围内是受保护的
		  *			CriticalSectionLocker  myCSLocker( &myCS );
		  *			...
		  *		}  // 结束保护区域
		  *  \endcode
		  *  \sa CriticalSection, THREADSAFE_OPERATION
		  */
		class BASE_IMPEXP CriticalSectionLocker
		{
		protected:
			const CriticalSection	*m_cs;

		public:
			/** Constructor: 进入临界区
			  * \note 指针可以为NULL,在这种情况下没有任何操作执行.
			  */
			CriticalSectionLocker(const CriticalSection *cs);

			CriticalSectionLocker(const CriticalSectionLocker &o) : m_cs(o.m_cs)
			{
			}

			CriticalSectionLocker & operator = (const CriticalSectionLocker&o)
			{
				m_cs = o.m_cs;
				return *this;
			}

			/** 析构函数:离开关键区.
			  */
			~CriticalSectionLocker();

		}; // end of CriticalSectionLocker



		/** 一个宏用来构建一个临界区用来保护一段代码，例如:
		  *  \code
		  *    CriticalSection  cs;
		  *    MyObject  obj;
		  *    ...
		  *
		  *    THREADSAFE_OPERATION(cs,  obj.foo(); )
		  *    ...
		  *    THREADSAFE_OPERATION(cs,  obj.foo(); obj.bar(); }
		  *
		  *  \endcode
		  *
		  * \sa CriticalSectionLocker, CThreadSafeVariable
		  */
		#define  THREADSAFE_OPERATION(_CRITSECT_OBJ, CODE_TO_EXECUTE )  \
				{ \
					mvg::synch::CriticalSectionLocker lock(&_CRITSECT_OBJ); \
					CODE_TO_EXECUTE \
				}


	} // End of namespace
} // End of namespace

#endif // MVG_SYNCH_CRITICAL_SECTION_H_
