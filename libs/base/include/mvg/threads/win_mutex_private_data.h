#ifndef MVG_THREADS_WIN_MUTEX_PRIVATE_DATA_H_
#define MVG_THREADS_WIN_MUTEX_PRIVATE_DATA_H_

#include <windows.h>

namespace mvg 
{
	namespace threads
	{
		class Win32MutexPrivateData 
		{
			friend class Mutex;
			friend class Condition;

		private:

			Win32MutexPrivateData();
			~Win32MutexPrivateData();

			CRITICAL_SECTION m_cs;
		};
	}
}

#endif // MVG_THREADS_WIN_MUTEX_PRIVATE_DATA_H_





