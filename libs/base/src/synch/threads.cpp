/*******************************************************************************
 * 文件： threads.cpp
 * 时间： 2014/12/14 19:11
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明：添加线程的一些公共方法
 *
********************************************************************************/
#include "base_precomp.h"
#include <mvg/synch/threads.h>

#include <conio.h>
#include <windows.h>
#include <process.h>
#include <tlhelp32.h>
#include <sys/utime.h>
#include <io.h>
#include <direct.h>

#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

#include <mvg/utils/mvg_macros.h>
#include <mvg/synch/semaphore.h>

/**当前线程sleep时间，单位毫秒，采用这种方式便于以后扩展
*/
void mvg::synch::sleep(int time_ms) MVG_NO_THROWS
{
	Sleep(time_ms);
}

/*---------------------------------------------------------------
createThread
---------------------------------------------------------------*/
namespace mvg
{
	namespace synch
	{
		struct TAuxThreadLaucher
		{
			TAuxThreadLaucher() : ptrFunc(NULL), param(NULL), win_sem(0, 10)
			{
			}
			void(*ptrFunc) (void *);
			void    *param;
			unsigned long		myWindowsId;
			synch::Semaphore	win_sem;
		};

		void *auxiliary_thread_launcher(void *param)
		{
			try
			{
				TAuxThreadLaucher   *d = reinterpret_cast<TAuxThreadLaucher*>(param);
				TAuxThreadLaucher	localCopy = *d;
				d->myWindowsId = (unsigned long)GetCurrentThreadId();
				d->win_sem.release();

				// 启动用户代码
				localCopy.ptrFunc(localCopy.param);
			}
			catch (std::exception &e)
			{
				std::cout << "Exception in [auxiliary_thread_launcher_LIN/WIN]!!!:\n" << e.what();
			}
			catch (...)
			{
				std::cout << "Untyped exception in [auxiliary_thread_launcher_LIN/WIN]!!!\n";
			}
			return NULL;
		}
		void auxiliary_thread_launcher_WIN(void *param)
		{
			auxiliary_thread_launcher(param);
		}

	} // end namespace
} // end namespace


mvg::synch::TThreadHandle mvg::synch::detail::createThreadImpl(
	void(*func)(void *),
	void       *param
	)
{
	MVG_START

	TAuxThreadLaucher   *auxData = new TAuxThreadLaucher();
	auxData->ptrFunc = func;
	auxData->param = param;

	TThreadHandle		threadHandle;

	HANDLE h = (HANDLE)_beginthread(auxiliary_thread_launcher_WIN, 0, auxData);
	if (h == ((HANDLE)-1))
	{
		delete auxData;
		THROW_EXCEPTION("Error creating new thread");
	}

	threadHandle.hThread = h;

	// 等待线程启动，这样我们就知道它的ID
	auxData->win_sem.waitForSignal();
	threadHandle.idThread = auxData->myWindowsId;

	delete auxData; auxData = NULL;

	return threadHandle;

	MVG_END
}


/** 等待给出的线程结束.
*/
void mvg::synch::joinThread(const TThreadHandle &threadHandle)
{
	if (threadHandle.isClear()) return;

	int prio = GetThreadPriority((HANDLE)threadHandle.hThread);
	if (THREAD_PRIORITY_ERROR_RETURN == prio)
		return; // 这边表示这个不是一个正在运行的线程

	DWORD ret = WaitForSingleObject((HANDLE)threadHandle.hThread, INFINITE);
	if (ret != WAIT_OBJECT_0)
		std::cerr << "[mvg::synch::joinThread] Error waiting for thread completion!" << std::endl;

}

/** 返回当前线程的ID
*/
unsigned long mvg::synch::getCurrentThreadId() MVG_NO_THROWS
{
	return GetCurrentThreadId();
}

/** 返回当前线程的handle
*/
mvg::synch::TThreadHandle mvg::synch::getCurrentThreadHandle() MVG_NO_THROWS
{
	mvg::synch::TThreadHandle h;
	h.hThread = GetCurrentThread();
	h.idThread = GetCurrentThreadId();
	return h;
}

/**更改给定线程的优先级
*/
void  mvg::synch::changeThreadPriority(
const TThreadHandle &threadHandle,
TThreadPriority priority)
{
	SetThreadPriority(threadHandle.hThread, priority);
}

/**更改给定进程的优先级（包括所有的线程）
*/
void  mvg::synch::changeCurrentProcessPriority(TProcessPriority priority)
{
	DWORD dwPri;
	switch (priority)
	{
	case ppIdle:	dwPri = IDLE_PRIORITY_CLASS; break;
	case ppNormal:	dwPri = NORMAL_PRIORITY_CLASS; break;
	case ppHigh:	dwPri = HIGH_PRIORITY_CLASS; break;
	case ppVeryHigh: dwPri = REALTIME_PRIORITY_CLASS; break;
	default:
		THROW_EXCEPTION("Invalid priority value");
	}
	SetPriorityClass(GetCurrentProcess(), dwPri);
}

/**返回当前线程创建到退出消耗的CPU时间
*/
void mvg::synch::getCurrentThreadTimes(
	time_t			&creationTime,
	time_t			&exitTime,
	double			&cpuTime)
{
	MVG_START

	FILETIME	timCreat, timExit, timKernel, timUser;
	uint64_t	t;

	HANDLE threadHandle;

	// 根据ID得到当前线程的handle
	threadHandle = OpenThread(READ_CONTROL | THREAD_QUERY_INFORMATION, FALSE, GetCurrentThreadId());  // threadId);
	if (!threadHandle)	 THROW_EXCEPTION("Cannot open the thread with the given 'threadId'");

	if (!GetThreadTimes(threadHandle, &timCreat, &timExit, &timKernel, &timUser))
	{
		CloseHandle(threadHandle);
		THROW_EXCEPTION("Error accessing thread times!");
	}

	CloseHandle(threadHandle);

	t = (((uint64_t)timCreat.dwHighDateTime) << 32) | timCreat.dwLowDateTime;
	creationTime = (t - 116444736000000000ULL) / 10000000;

	t = (((uint64_t)timExit.dwHighDateTime) << 32) | timExit.dwLowDateTime;
	exitTime = (t - 116444736000000000ULL) / 10000000;

	// CPU时间是用户+内核
	int64_t	t1 = (((uint64_t)timKernel.dwHighDateTime) << 32) | timKernel.dwLowDateTime;
	int64_t	t2 = (((uint64_t)timUser.dwHighDateTime) << 32) | timUser.dwLowDateTime;

	cpuTime = ((double)(t1 + t2)) * 100e-9;	// FILETIME counts intervals of 100ns


	MVG_END
}

/**执行给定命令（程序+参数），等待直到其结束
*/
bool mvg::synch::launchProcess(const std::string & command)
{
	STARTUPINFOA			SI;
	PROCESS_INFORMATION		PI;

	memset(&SI, 0, sizeof(STARTUPINFOA));
	SI.cb = sizeof(STARTUPINFOA);

	if (CreateProcessA(NULL, (LPSTR)command.c_str(), NULL, NULL, true, 0, NULL, NULL, &SI, &PI))
	{
		// Wait:
		WaitForSingleObject(PI.hProcess, INFINITE);
		return true;
	} // End of process executed OK
	else
	{
		char str[300];
		DWORD e = GetLastError();

		FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, 0, e, 0, str, sizeof(str), NULL);

		// ERROR:
		std::cerr << "[launchProcess] Couldn't spawn process. Error msg: " << str << std::endl;
		return false;
	}

} // end launchProcess


/** 显示关闭当前正在运行的线程，不要经常使用，最好在运行线程返回的时候使用
*/
void mvg::synch::exitThread() MVG_NO_THROWS
{
	ExitThread(0);
}

/** 终止线程，删除最后所有资源
*/
void mvg::synch::terminateThread(TThreadHandle &threadHandle) MVG_NO_THROWS
{
	if (threadHandle.isClear()) return; // done
	TerminateThread(threadHandle.hThread, -1);
	threadHandle.clear();
}