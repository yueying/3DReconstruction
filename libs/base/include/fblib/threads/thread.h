/*******************************************************************************
 * 文件： thread.h
 * 时间： 2015/04/14 17:19
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 多线程，Thread类为线程的实现类，是一个面向对象的线程实现接口，每派生一个Thread类
 *        并重新实现其代码，将相当于定义了一个共享进程资源但是可以独立调度的线程。
 *        通过重新run()和cancel()这两个成员函数，即可以实现线程运行时和取消时的操作；通过调用start()
 *        和cancel()，可以启动或终止已经定义的线程对象。
 *
********************************************************************************/
#ifndef FBLIB_THREADS_THREAD_H_
#define FBLIB_THREADS_THREAD_H_

#include <sys/types.h>

#include <fblib/threads/mutex.h>

namespace fblib 
{
	namespace threads
	{
		/**
		 *  得到处理器的个数，如果系统不支持，则返回1，目前windows下支持
		 */
		extern BASE_IMPEXP int GetNumberOfProcessors();

		/**
		 *  设置当前线程关联处理器
		 *  注意：系统不支持的情况下，返回-1
		 */
		extern BASE_IMPEXP int SetProcessorAffinityOfCurrentThread(unsigned int cpunum);

		/**
		 *  这个类提供了一个面向对象的线程接口
		 */
		class BASE_IMPEXP Thread {

		public:

			/**
			 *  Set the concurrency level for a running application.  This method
			 *  only has effect if the pthreads thread model is being used, and
			 *  then only when that model is many-to-one (eg. irix).
			 *  in other cases it is ignored.  The concurrency level is only a
			 *  *hint* as to the number of execution vehicles to use, the actual
			 *  implementation may do anything it wants.  Setting the value
			 *  to 0 returns things to their default state.
			 *
			 *  @return previous concurrency level, -1 indicates no-op.
			 */
			static int SetConcurrency(int concurrencyLevel);

			/**
			 *  Get the concurrency level for a running application.  In this
			 *  case, a return code of 0 means that the application is in default
			 *  mode.  A return code of -1 means that the application is incapable
			 *  of setting an arbitrary concurrency, because it is a one-to-one
			 *  execution model (sprocs, linuxThreads)
			 */
			static int GetConcurrency();

			/**
			 *  Enumerated Type for thread priority
			 */
			enum ThreadPriority {

				THREAD_PRIORITY_MAX,      /**< The maximum possible priority  */
				THREAD_PRIORITY_HIGH,     /**< A high (but not max) setting   */
				THREAD_PRIORITY_NOMINAL,  /**< An average priority            */
				THREAD_PRIORITY_LOW,      /**< A low (but not min) setting    */
				THREAD_PRIORITY_MIN,      /**< The miniumum possible priority */
				THREAD_PRIORITY_DEFAULT   /**< Priority scheduling default    */

			};

			/**
			 *  Enumerated Type for thread scheduling policy
			 */
			enum ThreadPolicy {

				THREAD_SCHEDULE_FIFO,        /**< First in, First out scheduling         */
				THREAD_SCHEDULE_ROUND_ROBIN, /**< Round-robin scheduling (LINUX_DEFAULT) */
				THREAD_SCHEDULE_TIME_SHARE,  /**< Time-share scheduling (IRIX DEFAULT)   */
				THREAD_SCHEDULE_DEFAULT      /**< Default scheduling                     */

			};

			/**
			 *  默认构造函数
			 */
			Thread();

			/**
			 *  析构函数
			 */
			virtual ~Thread();

			/**
			 *  返回一个指针指向当前正在运行的线程
			 */
			static Thread *CurrentThread();

			/**
			 *  线程初始化，这个方法必须最先执行
			 */
			static void Init();

			/**
			 *  要求当前线程出让CPU控制权，交给其他正在等待的线程
			 *
			 *  @note 如果这个方法应用到处理进程上面，就等同调用了sched_yield().
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			static int YieldCurrentThread();

			/**
			 *  This method will return the ThreadPriority of the master process.
			 *  (ie, the one calling the thread->start() methods for the first time)
			 *  The method will almost certainly return
			 *  Thread::THREAD_PRIORITY_DEFAULT if
			 *  Init() has not been called.
			 *
			 *  @return the Thread::ThreadPriority of the master thread.
			 */
			static ThreadPriority GetMasterPriority() { return s_masterThreadPriority; };


			/**
			 *  获得唯一的一个线程id，这个id单调递增
			 *
			 *  @return 一个唯一的线程标识
			 */
			int getThreadId();

			/**
			 *  Get the thread's process id.  This is the pthread_t or pid_t value
			 *  depending on the threading model being used.
			 *
			 *  @return thread process id.
			 */
			size_t getProcessId();

			/**
			 *  启动线程，该方法会配置线程设置线程优先级，自动执行线程的run()方法
			 *
			 *  @note  如果通过setStackSize指定栈的大小小于最小允许的栈的大小，
			 *  则线程栈的大小将设为最小允许栈的大小，可以通过getStackSize()获得。
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int start();
			int startThread();

			/**
			 * Test the cancel state of the thread.  If the thread has been canceled
			 * this method will cause the thread to exit now.  This method operates
			 * on the calling thread.
			 *
			 * Returns 0 if normal, -1 if called from a thread other that this.
			 */
			int testCancel();


			/**
			 *  虚函数，用于终止线程的执行，等价于 SIGKILL.
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			virtual int cancel();

			/**
			 *  Set the thread's schedule priority.  This is a complex method.
			 *  Beware of thread priorities when using a many-to-many kernel
			 *  entity implemenation (such as IRIX pthreads).  If one is not carefull
			 *  to manage the thread priorities, a priority inversion deadlock can
			 *  easily occur (Although the fblib::Mutex & fblib::Barrier
			 *  constructs have been designed with this senario in mind).  Unless
			 *  you have explicit need to set the schedule pirorites for a given
			 *  task, it is best to leave them alone.
			 *
			 *  @note some implementations (notably LinuxThreads and IRIX Sprocs)
			 *  only alow you to decrease thread priorities dynamically.  Thus,
			 *  a lower priority thread will not allow it's priority to be raised
			 *  on the fly.
			 *
			 *  @note seting the environment variable OUTPUT_THREADLIB_SCHEDULING_INFO
			 *  will output scheduling information for each thread to stdout.
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int setSchedulePriority(ThreadPriority priority);

			/**
			 *  Get the thread's schedule priority (if able)
			 *
			 *  @note seting the environment variable OUTPUT_THREADLIB_SCHEDULING_INFO
			 *  will output scheduling information for each thread to stdout.
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int getSchedulePriority();

			/**
			 *  Set the thread's scheduling policy (if able)
			 *
			 *  @note On some implementations (notably IRIX Sprocs & LinuxThreads)
			 *  The policy may prohibit the use of SCHEDULE_ROUND_ROBIN and
			 *  SCHEDULE_FIFO policies - due to their real-time nature, and
			 *  the danger of deadlocking the machine when used as super-user.
			 *  In such cases, the command is a no-op.
			 *
			 *  @note seting the environment variable OUTPUT_THREADLIB_SCHEDULING_INFO
			 *  will output scheduling information for each thread to stdout.
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int setSchedulePolicy(ThreadPolicy policy);

			/**
			 *  Get the thread's policy (if able)
			 *
			 *  @note seting the environment variable OUTPUT_THREADLIB_SCHEDULING_INFO
			 *  will output scheduling information for each thread to stdout.
			 *
			 *  @return policy if normal, -1 if errno set, errno code otherwise.
			 */
			int getSchedulePolicy();

			/**
			 *  Set the thread's desired stack size (in bytes).
			 *  This method is an attribute of the thread and must be called
			 *  *before* the start() method is invoked.
			 *
			 *  @note a return code of 13 (EACESS) means that the thread stack
			 *  size can no longer be changed.
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int setStackSize(size_t size);

			/**
			 *  Get the thread's desired stack size.
			 *
			 *  @return the thread's stack size.  0 indicates that the stack size
			 *   has either not yet been initialized, or not yet been specified by
			 *   the application.
			 */
			size_t getStackSize();

			/**
			 *  Print the thread's scheduling information to stdout.
			 */
			void printSchedulingInfo();

			/**
			 *  Detach the thread from the calling process.
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int detach();

			/**
			 *  Join the calling process with the thread
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int join();

			/**
			 *  Disable thread cancelation altogether. Thread::cancel() has no effect.
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int setCancelModeDisable();

			/**
			 *  Mark the thread to cancel aysncronously on Thread::cancel().
			 *  (May not be available with process-level implementations).
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int setCancelModeAsynchronous();

			/**
			 *  Mark the thread to cancel at the earliest convenience on
			 *  Thread::cancel() (This is the default)
			 *
			 *  @return 正常返回 0, 如果errno设置了返回-1或者其它errno code
			 */
			int setCancelModeDeferred();

			/**
			 *  查询线程运行状态
			 *
			 *  @return ture 表示运行, false 不运行.
			 */
			bool isRunning();

			/**
			 *  线程执行的主函数，纯虚函数，必须被派生类实现
			 *  在这个函数中可以循环执行一段线程功能代码，
			 *  但最后一定要使用YieldCurrentThread()出让CPU的控制权
			 */
			virtual void run() = 0;

			/**
			 *  Thread's cancel cleanup routine, called upon cancel(), after the
			 *  cancelation has taken place, but before the thread exits completely.
			 *  This method should be used to repair parts of the thread's data
			 *  that may have been damaged by a pre-mature cancel.  No-op by default.
			 */
			virtual void cancelCleanup() {};

			void* getImplementation(){ return m_prvData; };

			/** Thread's processor affinity method.  This binds a thread to a
			  * processor whenever possible.   This call must be made before
			  * start() or startThread() and has no effect after the thread
			  * has been running.  In the pthreads implementation, this is only
			  * implemented on sgi, through a pthread extension.  On other pthread
			  * platforms this is ignored.  Returns 0 on success, implementation's
			  * error on failure, or -1 if ignored.
			  */
			int setProcessorAffinity(unsigned int cpunum);

			/** microSleep method, equivilant to the posix usleep(microsec).
			  *  This is not strictly thread API but is used
			  * so often with threads. It's basically UNIX usleep. Parameter is
			  * number of microseconds we current thread to sleep. Returns 0 on
			  * succes, non-zero on failure (UNIX errno or GetLastError() will give
			  * detailed description.
			  */
			static int microSleep(unsigned int microsec);

		private:

			/**
			 *  The Private Actions class is allowed to operate on private data.
			 */
			friend class ThreadPrivateActions;

			/**
			 *  Private copy constructor, to prevent tampering.
			 */
			Thread(const Thread &/*t*/) {};

			/**
			  *  Private copy assignment, to prevent tampering.
			  */
			Thread &operator=(const Thread &/*t*/) { return *(this); };

			/**
			 *  Implementation-specific data
			 */
			void * m_prvData;

			/**
			 *  Master thread's priority, set by Thread::Init.
			 */
			static ThreadPriority s_masterThreadPriority;

			/**
			 *  Is initialized flag
			 */
			static bool s_isInitialized;
		};
	}
}

#endif // FBLIB_THREADS_THREAD_H_
