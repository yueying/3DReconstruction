/*******************************************************************************
 * 文件： atomic_counter.cpp
 * 时间： 2014/11/09 19:58
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 多线程中保证原子递增或者递减,这边可能不需要，目前C++11中已经支持通用线程库
 *
********************************************************************************/
#include "base_precomp.h"

#include <mvg/synch/atomic_counter.h>

#include <windows.h>

using namespace mvg::synch;

void AtomicCounter::operator++()
{
	InterlockedIncrement(&m_value);
}

AtomicCounter::atomic_num_t AtomicCounter::operator--()
{
	return InterlockedDecrement(&m_value);
}

AtomicCounter::operator AtomicCounter::atomic_num_t() const
{
	return static_cast<long const volatile &>(m_value);
}