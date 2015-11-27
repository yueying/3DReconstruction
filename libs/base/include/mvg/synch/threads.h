/*******************************************************************************
 * 文件： threads.h
 * 时间： 2014/12/14 13:16
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 对windows线程库做一个简化，这边注意虽然C++11已经做了相关工作，在example中已经给出了
 *        如果不跨平台，则直接使用windows的线程库，不过这边做一个封装，代码中尽可能不要直接出现windows下函数
 *        1.简化windows下线程的使用
 *        2.很有可能下一步要跨平台
 *        3.C++11标准在vs2012以上才可以使用，另外锁的性能较差
 *        4.代码中尽可能的使用这个头文件，不要直接使用windows线程库，或者使用c++11标准，不过最后不好使用其中的锁
 *
********************************************************************************/
#ifndef MVG_SYNCH_THREADS_H_
#define MVG_SYNCH_THREADS_H_

#include <mvg/base/link_pragmas.h>
#include <Windows.h>
#include <mvg/utils/mvg_stdint.h>
#include <mvg/utils/mvg_macros.h>

namespace mvg
{
	namespace synch
	{
		/**定义一个线程handle*/
		struct TThreadHandle
		{
			TThreadHandle() :  //!< 构造函数初始化，设定这个值，代表句柄还没有初始化
				hThread(NULL),
				idThread(0)
			{
			}

			/** 设置句柄无效
			*/
			void clear()
			{
				idThread = 0;
				hThread = NULL;
			}
			/** 返回true表示句柄没有初始化 */
			bool isClear() const { return idThread == 0; }

			void *hThread;		//!< 线程句柄
			uintptr_t idThread;		//!< 线程ID			
		};

		/** 设置进程优先级，主要用于跨平台，进程优先级主要在windows程序中.
		*/
		enum TProcessPriority {
			ppIdle = 0,
			ppNormal,
			ppHigh,
			ppVeryHigh
		};

		/**  设置线程优先级，主要用于跨平台
		*/
		enum TThreadPriority {
			tpLowests = -15,	// Win32: THREAD_PRIORITY_IDLE
			tpLower = -2,	// Win32: THREAD_PRIORITY_LOWEST
			tpLow = -1,		// Win32: THREAD_PRIORITY_BELOW_NORMAL
			tpNormal = 0,   // Win32: THREAD_PRIORITY_NORMAL
			tpHigh = 1, 	// Win32: THREAD_PRIORITY_ABOVE_NORMAL
			tpHigher = 2, 	// Win32: THREAD_PRIORITY_HIGHEST
			tpHighest = 15	// Win32: THREAD_PRIORITY_TIME_CRITICAL
		};

		/** 这边提供库内部使用的方法 */
		namespace detail	{
			TThreadHandle BASE_IMPEXP createThreadImpl(void(*func)(void *), void *param);
			template<typename T> class ThreadCreateFunctor	{	//主要这边的T应该按引用传址，不能为常量.
			public:
				void(*func)(T);
				T obj;
				inline ThreadCreateFunctor(void(*f)(T), T o) :func(f), obj(o)	{}
				inline static void createThreadAux(void *obj)	{
					ThreadCreateFunctor<T> *auxStruct = static_cast<ThreadCreateFunctor<T> *>(obj);
					auxStruct->func(auxStruct->obj);
					delete auxStruct;
				}
				inline static TThreadHandle createThread(void(*f)(T), T param)	{
					ThreadCreateFunctor *tcs = new ThreadCreateFunctor(f, param);
					return createThreadImpl(&createThreadAux, static_cast<void *>(tcs));
				}
			};
			// 这边进行模板特殊化，设置T=void*，更方便处理：
			template<> class ThreadCreateFunctor<void *>	{
			public:
				inline static TThreadHandle createThread(void(*f)(void *), void *param)	{
					return createThreadImpl(f, param);
				}
			};
			// 特殊情况，T不是为"void"：
			class ThreadCreateFunctorNoParams	{
			public:
				void(*func)(void);
				ThreadCreateFunctorNoParams(void(*f)(void)) : func(f) { }

				inline static void createThreadAux(void *f)	{
					ThreadCreateFunctorNoParams *d = static_cast<ThreadCreateFunctorNoParams*>(f);
					d->func(); // 调用用户函数
					delete d;
				}
				inline static TThreadHandle createThread(void(*f)(void))	{
					ThreadCreateFunctorNoParams *dat = new ThreadCreateFunctorNoParams(f);
					return createThreadImpl(&createThreadAux, static_cast<void*>(dat));
				}
			};
			//模板方法，作为一个线程运行一个非静态的方法
			template <class CLASS, class PARAM>
			class ThreadCreateObjectFunctor {
			public:
				typedef void (CLASS::*objectfunctor_t)(PARAM);
				CLASS *obj;
				objectfunctor_t func;
				PARAM p;
				inline ThreadCreateObjectFunctor(CLASS *o, objectfunctor_t f, PARAM param) :obj(o), func(f), p(param) {}
				inline static void createThreadAux(void *p)	{
					ThreadCreateObjectFunctor<CLASS, PARAM> *auxStruct = static_cast<ThreadCreateObjectFunctor<CLASS, PARAM>*>(p);
					objectfunctor_t f = auxStruct->func;
					(auxStruct->obj->*f)(auxStruct->p);
					delete auxStruct;
				}
				inline static TThreadHandle createThread(CLASS *o, objectfunctor_t f, PARAM param)	{
					ThreadCreateObjectFunctor *tcs = new ThreadCreateObjectFunctor(o, f, param);
					return createThreadImpl(&createThreadAux, static_cast<void *>(tcs));
				}
			};
			// 模板方法，作为一个线程运行一个非静态的方法，不传参数
			template <class CLASS>
			class ThreadCreateObjectFunctorNoParams {
			public:
				typedef void (CLASS::*objectfunctor_t)(void);
				CLASS *obj;
				objectfunctor_t func;
				inline ThreadCreateObjectFunctorNoParams(CLASS *o, objectfunctor_t f) :obj(o), func(f) {}
				inline static void createThreadAux(void *p)	{
					ThreadCreateObjectFunctorNoParams<CLASS> *auxStruct = static_cast<ThreadCreateObjectFunctorNoParams<CLASS>*>(p);
					objectfunctor_t f = auxStruct->func;
					(auxStruct->obj->*f)();
					delete auxStruct;
				}
				inline static TThreadHandle createThread(CLASS *o, objectfunctor_t f)	{
					ThreadCreateObjectFunctorNoParams *tcs = new ThreadCreateObjectFunctorNoParams(o, f);
					return createThreadImpl(&createThreadAux, static_cast<void *>(tcs));
				}
			};
		} // end detail

		/**创建一个线程，传入一个函数名或静态方法以及一般参数.返回线程handle
		*/
		template<typename T> inline TThreadHandle createThread(void(*func)(T), T param)	{
			return detail::ThreadCreateFunctor<T>::createThread(func, param);
		}
		//! \overload
		template<typename T> inline TThreadHandle createThreadRef(void(*func)(T&), T& param)	{
			return detail::ThreadCreateFunctor<T&>::createThread(func, param);
		}
		//! \overload
		inline TThreadHandle createThread(void(*func)(void))	{
			return detail::ThreadCreateFunctorNoParams::createThread(func);
		}

		/** 创建一个线程运行一个非静态的方法，我们通过this访问，具体事例如下：
		*  \code
		*    class MyClass {
		*    public:
		*      void myThread(int n);
		*      void someMethod() {
		*         createThreadFromObjectMethod(this, &MyClass::myThread, 123 );
		*         ....
		*      }
		*    };
		*  \endcode
		*/
		template <typename CLASS, typename PARAM>
		inline TThreadHandle createThreadFromObjectMethod(CLASS *obj, void (CLASS::*func)(PARAM), PARAM param)	{
			return detail::ThreadCreateObjectFunctor<CLASS, PARAM>::createThread(obj, func, param);
		}
		//! \overload
		template <typename CLASS, typename PARAM>
		inline TThreadHandle createThreadFromObjectMethodRef(CLASS *obj, void (CLASS::*func)(PARAM&), PARAM &param)	{
			return detail::ThreadCreateObjectFunctor<CLASS, PARAM&>::createThread(obj, func, param);
		}
		//! \overload
		template <typename CLASS>
		inline TThreadHandle createThreadFromObjectMethod(CLASS *obj, void (CLASS::*func)(void))	{
			return detail::ThreadCreateObjectFunctorNoParams<CLASS>::createThread(obj, func);
		}

		/** 等待给出的线程结束.
		*/
		void BASE_IMPEXP joinThread(const TThreadHandle &threadHandle);

		/** 返回当前线程的ID
		*/
		unsigned long BASE_IMPEXP getCurrentThreadId() MVG_NO_THROWS;

		/** 返回当前线程的handle
		*/
		TThreadHandle BASE_IMPEXP getCurrentThreadHandle() MVG_NO_THROWS;

		/** 显示关闭当前正在运行的线程，不要经常使用，最好在运行线程返回的时候使用
		*/
		void BASE_IMPEXP exitThread() MVG_NO_THROWS;

		/**返回当前线程创建到退出消耗的CPU时间
		*/
		void BASE_IMPEXP getCurrentThreadTimes(
			time_t			&creationTime,
			time_t			&exitTime,
			double			&cpuTime);

		/**更改给定线程的优先级
		*/
		void BASE_IMPEXP changeThreadPriority(const TThreadHandle &threadHandle, TThreadPriority priority);

		/** 终止线性，删除最后所有资源
		*/
		void BASE_IMPEXP terminateThread(TThreadHandle &threadHandle) MVG_NO_THROWS;

		/**更改给定进程的优先级（包括所有的线程）
		*/
		void BASE_IMPEXP changeCurrentProcessPriority(TProcessPriority priority);

		/**当前线程sleep时间，单位毫秒，采用这种方式便于以后扩展
		*/
		void BASE_IMPEXP sleep(int time_ms) MVG_NO_THROWS;

		/**执行给定命令（程序+参数），等待直到其结束
		*/
		bool BASE_IMPEXP  launchProcess(const std::string & command);

	}
}

#endif // MVG_SYNCH_THREADS_H_