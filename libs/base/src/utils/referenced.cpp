#include "base_precomp.h"
#include <stdlib.h>

#include <fblib/utils/referenced.h>
#include <fblib/utils/notify.h>


#include <typeinfo>
#include <memory>
#include <set>

#include <fblib/threads/scoped_lock.h>
#include <fblib/threads/mutex.h>

#include <fblib/utils/delete_handler.h>

namespace fblib
{
	namespace utils{
		//#define ENFORCE_THREADSAFE
		//#define DEBUG_OBJECT_ALLOCATION_DESTRUCTION

		// specialized smart pointer, used to get round auto_ptr<>'s lack of the destructor reseting itself to 0.
		template<typename T>
		struct ResetPointer
		{
			ResetPointer() :
				m_ptr(0) {}

			ResetPointer(T* ptr) :
				m_ptr(ptr) {}

			~ResetPointer()
			{
				delete m_ptr;
				m_ptr = 0;
			}

			inline ResetPointer& operator = (T* ptr)
			{
				if (m_ptr == ptr) return *this;
				delete m_ptr;
				m_ptr = ptr;
				return *this;
			}

			void reset(T* ptr)
			{
				if (m_ptr == ptr) return;
				delete m_ptr;
				m_ptr = ptr;
			}

			inline T& operator*()  { return *m_ptr; }

			inline const T& operator*() const { return *m_ptr; }

			inline T* operator->() { return m_ptr; }

			inline const T* operator->() const   { return m_ptr; }

			T* get() { return m_ptr; }

			const T* get() const { return m_ptr; }

			T* m_ptr;
		};

		typedef ResetPointer<DeleteHandler> DeleteHandlerPointer;
		typedef ResetPointer<fblib::threads::Mutex> GlobalMutexPointer;

		fblib::threads::Mutex* Referenced::getGlobalReferencedMutex()
		{
			static GlobalMutexPointer s_ReferencedGlobalMutext = new fblib::threads::Mutex;
			return s_ReferencedGlobalMutext.get();
		}

		// helper class for forcing the global mutex to be constructed when the library is loaded.
		struct InitGlobalMutexes
		{
			InitGlobalMutexes()
			{
				Referenced::getGlobalReferencedMutex();
			}
		};
		static InitGlobalMutexes s_initGlobalMutexes;

		// static std::auto_ptr<DeleteHandler> s_deleteHandler(0);
		static DeleteHandlerPointer s_deleteHandler(0);

		void Referenced::setDeleteHandler(DeleteHandler* handler)
		{
			s_deleteHandler.reset(handler);
		}

		DeleteHandler* Referenced::getDeleteHandler()
		{
			return s_deleteHandler.get();
		}

		Referenced::Referenced() :
			m_refCount(0)
		{
		}

		Referenced::Referenced(bool threadSafeRefUnref) :
			m_refCount(0)
		{
		}

		Referenced::Referenced(const Referenced&) :
			m_refCount(0)
		{
		}

		Referenced::~Referenced()
		{
			if (m_refCount > 0)
			{
				FBLIB_WARN << "Warning: deleting still referenced object " << this << " of type '" << typeid(this).name() << "'" << std::endl;
				FBLIB_WARN << "         the final reference count was " << m_refCount << ", memory corruption possible." << std::endl;
			}		
		}

		void Referenced::setThreadSafeRefUnref(bool threadSafe)
		{
		}

		int Referenced::unref_nodelete() const
		{
			return --m_refCount;
		}

		void Referenced::deleteUsingDeleteHandler() const
		{
			getDeleteHandler()->requestDelete(this);
		}
	}
} // end of namespace osg
