/*******************************************************************************
 * 文件： Semaphore.cpp
 * 时间： 2014/11/13 12:57
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： windows下的信号量处理
 *
********************************************************************************/
#include "base_precomp.h"  // 预编译头
#include <mvg/synch/Semaphore.h>
#include <windows.h>
#include <mvg/utils/mvg_macros.h>

using namespace mvg::synch;

/*---------------------------------------------------------------
						Semaphore
---------------------------------------------------------------*/
Semaphore::Semaphore(
    unsigned int    initialCount,
    unsigned int    maxCount,
    const std::string &name )
    :
    m_name(name)
{
	MVG_START

	HANDLE hSem = CreateSemaphoreA(
		NULL,			// 表示安全控制，一般直接传入NULL
		initialCount,	// 表示初始资源数量
		maxCount,		// 表示最大并发数量
		name.size()==0 ? NULL : name.c_str() );

	if (!hSem)	THROW_EXCEPTION("Error creating semaphore!");

	m_data.resize( sizeof(HANDLE) );

	* m_data.getAs<HANDLE*>() = hSem;

	MVG_END
}

/*---------------------------------------------------------------
						~Semaphore
---------------------------------------------------------------*/
Semaphore::~Semaphore()
{
	if (m_data.use_count() == 1)
	{
		CloseHandle( * m_data.getAs<HANDLE*>() );
	}
}

/*---------------------------------------------------------------
阻塞，直到信号量非0
\param timeout_ms 设置超时时间（单位毫秒）,或者设置为0无限等待.
\return true信号量设置成功, false表示超时或者其它错误.
---------------------------------------------------------------*/
bool Semaphore::waitForSignal( unsigned int timeout_ms )
{
	MVG_START

	DWORD tim = (timeout_ms==0) ? INFINITE : timeout_ms;
	DWORD ret = WaitForSingleObject( * m_data.getAs<HANDLE*>(), tim );

	return (ret==WAIT_OBJECT_0);

	MVG_END
}

/*---------------------------------------------------------------
	增加信号量的计数.
---------------------------------------------------------------*/
void Semaphore::release(unsigned int increaseCount )
{
	MVG_START

	if (!ReleaseSemaphore(
		*m_data.getAs<HANDLE*>(),		// 信号量的句柄
		increaseCount,		// 表示增加个数，必须大于0且不超过最大资源数量
		NULL ))				// 可以用来传出先前的资源计数，设为NULL表示不需要传出
	THROW_EXCEPTION("Error increasing semaphore count!");

	MVG_END
}


