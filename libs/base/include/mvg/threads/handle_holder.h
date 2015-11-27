#ifndef MVG_THREADS_HANDLE_HOLDER_H_
#define MVG_THREADS_HANDLE_HOLDER_H_

#include <windows.h>

/************************************************************************/
/* Class that holds HANDLES ensuring proper destruction                 */
/* It is design decision to make this class noncopyable.                */    
/* It makes design much cleaner. If one wants to copy handle than one   */
/* can do hv.set(DuplicateHandle(....))                                 */
/************************************************************************/
namespace mvg {
	namespace threads{
		class HandleHolder{
		private:
			HANDLE m_handle;

			inline void close(){
				if (m_handle != INVALID_HANDLE_VALUE) CloseHandle(m_handle);
				m_handle = INVALID_HANDLE_VALUE;
			};

			// copy constructor - disallow
			HandleHolder(const HandleHolder& rhs);

			// assignment operator - disallow
			HandleHolder& operator=(const HandleHolder& rhs);

		public:
			// constructor
			HandleHolder()
				:m_handle(INVALID_HANDLE_VALUE)
			{};

			// constructor from HANDLE  
			explicit HandleHolder(HANDLE h)
				:m_handle(h)
			{};

			// destructor - CloseHandle()
			~HandleHolder()
			{
				close();
			};

			// accessor    
			const HANDLE& get() const {
				return m_handle;
			}

			// mutator
			void set(HANDLE h) {
				if (m_handle != INVALID_HANDLE_VALUE) close();
				m_handle = h;
			}

			operator bool(){
				return m_handle != INVALID_HANDLE_VALUE && m_handle != NULL;
			};

		};
	}
} // namespace

#endif // MVG_THREADS_HANDLE_HOLDER_H_
