/*******************************************************************************
 * 文件： CriticalSection.cpp
 * 时间： 2014/11/13 14:01
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 多线程中关键段（临界区）的封装
 *
********************************************************************************/

#include "base_precomp.h"  // 预编译头

#include <fblib/synch/critical_section.h>
#include <windows.h>
#include <fblib/utils/fblib_macros.h>

using namespace fblib::synch;

/**	定义临界区
 */
struct CRIT_SECT_WIN
{
	CRITICAL_SECTION	cs;
	unsigned long		currentThreadOwner;
};

/*---------------------------------------------------------------
						构造函数
---------------------------------------------------------------*/
CriticalSection::CriticalSection( const char *name )
{
	m_data.resize( sizeof(CRIT_SECT_WIN) + 30 );
	//定义关键段变量后必须先初始化
	InitializeCriticalSection( & m_data.getAs<CRIT_SECT_WIN*>()->cs );

	if (name!=NULL)
		m_name = name;
	else
		m_name = "Unnamed";

}

/*---------------------------------------------------------------
						析构函数
---------------------------------------------------------------*/
CriticalSection::~CriticalSection()
{
	if (m_data.use_count() == 1)
	{
		//用完之后记得销毁，这边在析构函数中
		DeleteCriticalSection( & m_data.getAs<CRIT_SECT_WIN*>()->cs );

		m_data.reset();
	}
}

/*---------------------------------------------------------------
						进入关键区域
---------------------------------------------------------------*/
void  CriticalSection::enter() const
{
	const unsigned long threadid = GetCurrentThreadId();

	CRIT_SECT_WIN *myCS = const_cast<CRIT_SECT_WIN  *>(  m_data.getAs<const CRIT_SECT_WIN*>() );

	if( myCS->currentThreadOwner == threadid )
		THROW_EXCEPTION(format("Detected recursive lock on critical section ('%s') by the same thread: %lu",m_name.c_str(),threadid ) )

	EnterCriticalSection( & myCS->cs );

	ASSERT_( myCS->currentThreadOwner == 0 );
	myCS->currentThreadOwner = threadid;

}

/*---------------------------------------------------------------
						leave
---------------------------------------------------------------*/
void  CriticalSection::leave() const
{
	const unsigned long threadid = GetCurrentThreadId();

	CRIT_SECT_WIN *myCS = const_cast<CRIT_SECT_WIN  *>(  m_data.getAs<const CRIT_SECT_WIN*>() );

	if ( myCS->currentThreadOwner!=threadid )
		THROW_EXCEPTION(format("Trying to release a critical section  ('%s') locked by a different thread.",m_name.c_str()));

	myCS->currentThreadOwner = 0;

	LeaveCriticalSection( & myCS->cs );
}

