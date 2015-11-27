/*******************************************************************************
 * 文件： win_thread_barrier.cpp
 * 时间： 2015/04/16 15:21
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 构造栅栏类，跟POSIX类似
 *
********************************************************************************/
#include "base_precomp.h"
#include  <mvg/threads/barrier.h>
#include  <mvg/threads/thread.h>
#include  <mvg/threads/scoped_lock.h>
#include <mvg/threads/win_barrier_private_data.h>
using namespace mvg::threads;

// so compiler can place it somewhere
Win32BarrierPrivateData::~Win32BarrierPrivateData()
{
}

//----------------------------------------------------------------------------
//
// Description: Constructor
//
// Use: public.
//
Barrier::Barrier(int numThreads) {
    Win32BarrierPrivateData *pd = new Win32BarrierPrivateData(numThreads, 0, 0);
    _valid = true;
    m_prvData = static_cast<void *>(pd);
}
//----------------------------------------------------------------------------
//
// Description: Destructor
//
// Use: public.
//
Barrier::~Barrier() {
    Win32BarrierPrivateData *pd =
        static_cast<Win32BarrierPrivateData*>(m_prvData);
    delete pd;
}
//----------------------------------------------------------------------------
//
// Description: Reset the barrier to its original state
//
// Use: public.
//
void Barrier::reset() {
    Win32BarrierPrivateData *pd =
        static_cast<Win32BarrierPrivateData*>(m_prvData);
    pd->cnt = 0;
    pd->phase = 0;
}
//----------------------------------------------------------------------------
//
// Description: Block until numThreads threads have entered the barrier.
//
// Use: public.
//
void Barrier::block(unsigned int numThreads) {

    Win32BarrierPrivateData *pd =
        static_cast<Win32BarrierPrivateData*>(m_prvData);

    if(numThreads != 0) pd->maxcnt = numThreads;
    int my_phase;

    ScopedLock<Mutex> lock(pd->lock);
    if( _valid )
    {
        my_phase = pd->phase;
        ++pd->cnt;

        if (pd->cnt == pd->maxcnt) {             // I am the last one
            pd->cnt = 0;                         // reset for next use
            pd->phase = 1 - my_phase;            // toggle phase
            pd->cond.broadcast();
        }else{ 
            while (pd->phase == my_phase ) {
                pd->cond.wait(&pd->lock);
            }
        }
    }
}

void Barrier::invalidate()
{
    Win32BarrierPrivateData *pd =
            static_cast<Win32BarrierPrivateData*>(m_prvData);

    pd->lock.lock();
    _valid = false;
    pd->lock.unlock();
    release();
}


//----------------------------------------------------------------------------
//
// Description: Release the barrier, now.
//
// Use: public.
//
void Barrier::release() {

    Win32BarrierPrivateData *pd =
        static_cast<Win32BarrierPrivateData*>(m_prvData);

    int my_phase;

    ScopedLock<Mutex> lock(pd->lock);
    my_phase = pd->phase;
    
    pd->cnt = 0;                         // reset for next use
    pd->phase = 1 - my_phase;            // toggle phase
    pd->cond.broadcast();
    
}

//----------------------------------------------------------------------------
//
// Description: Return the number of threads currently blocked in the barrier
//
// Use: public
//
int Barrier::numThreadsCurrentlyBlocked() {
    
    Win32BarrierPrivateData *pd =
        static_cast<Win32BarrierPrivateData*>(m_prvData);

    int numBlocked = -1;
    ScopedLock<Mutex> lock(pd->lock);
    numBlocked = pd->cnt;
    return numBlocked;

}