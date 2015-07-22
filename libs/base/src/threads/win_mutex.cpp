/*******************************************************************************
 * 文件： win_mutex.cpp
 * 时间： 2015/04/16 19:32
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 互斥体
 *
********************************************************************************/
#include "base_precomp.h"
#include <fblib/threads/mutex.h>
#include <fblib/threads/win_mutex_private_data.h>

namespace fblib
{
	namespace threads
	{

		Win32MutexPrivateData::Win32MutexPrivateData()
		{
			InitializeCriticalSection(&m_cs);
		}
		Win32MutexPrivateData::~Win32MutexPrivateData()
		{
			DeleteCriticalSection(&m_cs);
		}

		/**	构造函数
		 */
		Mutex::Mutex(MutexType type) :
			m_mutexType(type)
		{
			Win32MutexPrivateData *pd = new Win32MutexPrivateData();
			m_prvData = static_cast<void *>(pd);
		}

		/**	析构函数
		 */
		Mutex::~Mutex() {
			delete static_cast<Win32MutexPrivateData*>(m_prvData);
		}
		//----------------------------------------------------------------------------
		//
		// Description: lock the mutex
		//
		// Use: public.
		//
		int Mutex::lock() 
		{
			Win32MutexPrivateData *pd =
				static_cast<Win32MutexPrivateData*>(m_prvData);

			// Block until we can take this lock.
			EnterCriticalSection(&(pd->m_cs));

			return 0;
		}

		//----------------------------------------------------------------------------
		//
		// Description: unlock the mutex
		//
		// Use: public.
		//
		int Mutex::unlock() {
			Win32MutexPrivateData *pd =
				static_cast<Win32MutexPrivateData*>(m_prvData);

			// Release this lock. CRITICAL_SECTION is nested, thus
			//   unlock() calls must be paired with lock() calls.
			LeaveCriticalSection(&(pd->m_cs));

			return 0;

		}

		//----------------------------------------------------------------------------
		//
		// Description: test if the mutex may be locked
		//
		// Use: public.
		//
		int Mutex::trylock() 
		{
			Win32MutexPrivateData *pd =
				static_cast<Win32MutexPrivateData*>(m_prvData);

			// Take the lock if we can; regardless don't block.
			// 'result' is FALSE if we took the lock or already held
			//   it amd TRUE if another thread already owns the lock.
			BOOL result = TryEnterCriticalSection(&(pd->m_cs));

			return((result == TRUE) ? 0 : 1);
		}
	}
}