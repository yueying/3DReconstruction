#ifndef FBLIB_THREADS_WIN_MUTEX_PRIVATE_DATA_H_
#define FBLIB_THREADS_WIN_MUTEX_PRIVATE_DATA_H_

#include <windows.h>

namespace fblib 
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

#endif // FBLIB_THREADS_WIN_MUTEX_PRIVATE_DATA_H_





