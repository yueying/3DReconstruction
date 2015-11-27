#include "base_precomp.h"
#include <mvg/threads/condition.h>
#include  <mvg/threads/thread.h>
#include <mvg/threads/win_condition_private_data.h>

namespace mvg
{
	namespace threads
	{
		Win32ConditionPrivateData::~Win32ConditionPrivateData()
		{
		}

		/**
		*  构造函数
		*/
		Condition::Condition() {
			Win32ConditionPrivateData *pd =
				new Win32ConditionPrivateData();
			m_prvData = static_cast<void *>(pd);
		}
		/**
		*  析构函数
		*/
		Condition::~Condition() {
			Win32ConditionPrivateData *pd =
				static_cast<Win32ConditionPrivateData *>(m_prvData);

			delete pd;
		}

		/**
		*  设置作为条件量的互斥体，并强制线程等待此条件满足
		*/
		int Condition::wait(Mutex *mutex) {

			Win32ConditionPrivateData *pd =
				static_cast<Win32ConditionPrivateData *>(m_prvData);

			return pd->wait(*mutex, INFINITE);
		}
		
		/**	设置作为条件量的互斥体，并强制线程等待此条件满足，并添加等待时间限制
		 */
		int Condition::wait(Mutex *mutex, unsigned long ms) {

			Win32ConditionPrivateData *pd =
				static_cast<Win32ConditionPrivateData *>(m_prvData);

			return pd->wait(*mutex, ms);
		}
		
		/**	唤醒一个线程
		 */
		int Condition::signal() {

			Win32ConditionPrivateData *pd =
				static_cast<Win32ConditionPrivateData *>(m_prvData);
			return pd->signal();
		}
		
		/**	唤醒所有被阻塞的线程
		 */
		int Condition::broadcast() {

			Win32ConditionPrivateData *pd =
				static_cast<Win32ConditionPrivateData *>(m_prvData);
			return pd->broadcast();
		}
	}
}