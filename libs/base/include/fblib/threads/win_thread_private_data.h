#ifndef FBLIB_THREADS_WIN_THREAD_PRIVATE_DATA_H_
#define FBLIB_THREADS_WIN_THREAD_PRIVATE_DATA_H_

#include <fblib/threads/thread.h>
#include <fblib/threads/block.h>
#include <fblib/threads/handle_holder.h>

namespace fblib 
{
	namespace threads
	{
		class Win32ThreadPrivateData 
		{
			// 对Thread类友好，则其可以访问我们的数据
			friend class Thread;
			// 对Win32PrivateActions类友好，则其可以访问我们的数据
			friend class ThreadPrivateActions;

		private:

			Win32ThreadPrivateData();
			~Win32ThreadPrivateData();

			size_t stackSize;
			bool isRunning;

			Block threadStartedBlock;

			int  cancelMode; // 0 - deffered (default) 1-asynch 2-disabled  

			bool detached;

			Thread::ThreadPriority threadPriority;

			Thread::ThreadPolicy threadPolicy;

			HandleHolder tid;

			int uniqueId;

			int cpunum;

		public:

			HandleHolder cancelEvent;

			struct TlsHolder{ // thread local storage slot
				DWORD getId()
				{
					if (!initialized) {
						ID = TlsAlloc();
						initialized = true;
					}
					return ID;
				}
				TlsHolder() : ID(TLS_OUT_OF_INDEXES), initialized(false) {}
				~TlsHolder(){
					if (initialized)
						TlsFree(ID);
					ID = TLS_OUT_OF_INDEXES;
				}
			private:
				DWORD ID;
				bool initialized;
			};

			static TlsHolder TLS;

		};

		DWORD cooperativeWait(HANDLE waitHandle, unsigned long timeout);

	}
}


#endif // FBLIB_THREADS_WIN_THREAD_PRIVATE_DATA_H_



