#ifndef MVG_THREADS_WIN_CONDITION_PRIVATE_DATA_H_
#define MVG_THREADS_WIN_CONDITION_PRIVATE_DATA_H_

#include <mvg/threads/scoped_lock.h>

#include <mvg/threads/win_thread_private_data.h>
#include <mvg/threads/handle_holder.h>

#define InterlockedGet(x) InterlockedExchangeAdd(x,0)

namespace mvg 
{
	namespace threads
	{
		class Condition;

		class Win32ConditionPrivateData 
		{
		public:
			friend class Condition;
			/// number of waiters.
			long waiters_;

			Win32ConditionPrivateData()
				:waiters_(0),
				sema_(CreateSemaphore(NULL, 0, 0x7fffffff, NULL)),
				waiters_done_(CreateEvent(NULL, FALSE, FALSE, NULL)),
				was_broadcast_(0)
			{
			}

			~Win32ConditionPrivateData();

			inline int broadcast()
			{
				int have_waiters = 0;
				long w = InterlockedGet(&waiters_);

				if (w > 0)
				{
					// we are broadcasting.  
					was_broadcast_ = 1;
					have_waiters = 1;
				}

				int result = 0;
				if (have_waiters)
				{
					// Wake up all the waiters.
					ReleaseSemaphore(sema_.get(), w, NULL);

					cooperativeWait(waiters_done_.get(), INFINITE);

					//end of broadcasting
					was_broadcast_ = 0;
				}
				return result;
			}

			inline int signal()
			{
				long w = InterlockedGet(&waiters_);
				int have_waiters = w > 0;

				int result = 0;

				if (have_waiters)
				{
					if (!ReleaseSemaphore(sema_.get(), 1, NULL))
						result = -1;
				}
				return result;
			}

			inline int wait(Mutex& external_mutex, long timeout_ms)
			{

				// Prevent race conditions on the <waiters_> count.
				InterlockedIncrement(&waiters_);

				int result = 0;

				ReverseScopedLock<Mutex> lock(external_mutex);

				// wait in timeslices, giving testCancel() a change to
				// exit the thread if requested.
				try {
					DWORD dwResult = cooperativeWait(sema_.get(), timeout_ms);
					if (dwResult != WAIT_OBJECT_0)
						result = (int)dwResult;
				}
				catch (...){
					// thread is canceled in cooperative wait , do cleanup
					long w = InterlockedDecrement(&waiters_);
					int last_waiter = was_broadcast_ && w == 0;

					if (last_waiter)  SetEvent(waiters_done_.get());
					// rethrow
					throw;
				}


				// We're ready to return, so there's one less waiter.
				long w = InterlockedDecrement(&waiters_);
				int last_waiter = was_broadcast_ && w == 0;

				if (result != -1 && last_waiter)
					SetEvent(waiters_done_.get());

				return result;
			}

		protected:

			/// Serialize access to the waiters count.
			/// Mutex waiters_lock_;
			/// Queue up threads waiting for the condition to become signaled.
			HandleHolder sema_;
			/**
			 * An auto reset event used by the broadcast/signal thread to wait
			 * for the waiting thread(s) to wake up and get a chance at the
			 * semaphore.
			 */
			HandleHolder waiters_done_;
			/// Keeps track of whether we were broadcasting or just signaling.
			size_t was_broadcast_;
		};

#undef InterlockedGet
	}
}


#endif // MVG_THREADS_WIN_CONDITION_PRIVATE_DATA_H_



