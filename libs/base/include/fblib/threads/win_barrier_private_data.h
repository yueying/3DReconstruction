#ifndef FBLIB_THREADS_WIN_BARRIER_PRIVATE_DATA_H_
#define FBLIB_THREADS_WIN_BARRIER_PRIVATE_DATA_H_

#include <fblib/threads/mutex.h>
#include <fblib/threads/condition.h>

namespace fblib 
{
	namespace threads
	{
		class Barrier;

		class Win32BarrierPrivateData {

			friend class Barrier;
		private:
			Win32BarrierPrivateData(int mc, int c, int p) :
				maxcnt(mc), cnt(c), phase(p) {}

			~Win32BarrierPrivateData();

			Condition cond;            // cv for waiters at barrier

			Mutex    lock;            // mutex for waiters at barrier

			volatile int       maxcnt;          // number of threads to wait for

			volatile int       cnt;             // number of waiting threads

			volatile int       phase;           // flag to seperate two barriers


		};

	}
}

#endif // FBLIB_THREADS_WIN_BARRIER_PRIVATE_DATA_H_




