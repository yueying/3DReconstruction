#include "base_precomp.h"
#include <mvg/threads/atomic.h>

#include <windows.h>
#include <intrin.h>
#pragma intrinsic(_InterlockedAnd)
#pragma intrinsic(_InterlockedOr)
#pragma intrinsic(_InterlockedXor)


namespace mvg {
	namespace threads{
		unsigned Atomic::operator++()
		{
			return InterlockedIncrement(&m_value);
		}

		unsigned Atomic::operator--()
		{
			return InterlockedDecrement(&m_value);
		}

		unsigned Atomic::AND(unsigned value)
		{
			return _InterlockedAnd(&m_value, value);
		}

		unsigned Atomic::OR(unsigned value)
		{
			return _InterlockedOr(&m_value, value);
		}

		unsigned Atomic::XOR(unsigned value)
		{
			return _InterlockedXor(&m_value, value);
		}


		unsigned Atomic::exchange(unsigned value)
		{
			return InterlockedExchange(&m_value, value);
		}


		Atomic::operator unsigned() const
		{
			MemoryBarrier();//限制指令重排和内存读写的缓存
			return m_value;
		}


		bool AtomicPtr::assign(void* ptrNew, const void* const ptrOld)
		{
			return ptrOld == InterlockedCompareExchangePointer((PVOID volatile*)&m_ptr, (PVOID)ptrNew, (PVOID)ptrOld);
		}

		void* AtomicPtr::get() const
		{
			MemoryBarrier();
			return m_ptr;
		}
	}
}