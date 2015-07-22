/*******************************************************************************
 * 文件： CriticalSectionLocker.cpp
 * 时间： 2014/11/13 14:11
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明：  一个类进入临界区在构造函数中，离开在析构函数中.
 *
********************************************************************************/
#include "base_precomp.h"  // 预编译头

#include <fblib/synch/critical_section.h>

using namespace fblib::synch;

/*---------------------------------------------------------------
				CriticalSectionLocker
---------------------------------------------------------------*/
CriticalSectionLocker::CriticalSectionLocker( const CriticalSection * cs)
	: m_cs(cs)
{
	if (m_cs)
	{
		m_cs->enter();
	}
}

/*---------------------------------------------------------------
				~CriticalSectionLocker
---------------------------------------------------------------*/
CriticalSectionLocker::~CriticalSectionLocker()
{
	if (m_cs)
	{
		m_cs->leave();
	}
}
