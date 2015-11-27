#include "base_precomp.h"
#include "mvg/utils/aligned_malloc.h"
#include "mvg/utils/notify.h"

#if !defined(__APPLE__) && !defined(__FreeBSD__) && !defined(__NetBSD__)
// Needed for memalign on Linux and _aligned_alloc on Windows.
#  ifdef FREE_WINDOWS
/* make sure _aligned_malloc is included */
#    ifdef __MSVCRT_VERSION__
#      undef __MSVCRT_VERSION__
#    endif

#    define __MSVCRT_VERSION__ 0x0700
#  endif  // FREE_WINDOWS

#  include <malloc.h>
#else
// Apple's malloc is 16-byte aligned, and does not have malloc.h, so include
// stdilb instead.
#  include <cstdlib>
#endif

namespace mvg {
	namespace utils{
		void *aligned_malloc(int size, int alignment) {
#ifdef _WIN32
			return _aligned_malloc(size, alignment);
#elif __APPLE__
			// On Mac OS X, both the heap and the stack are guaranteed 16-byte aligned so
			// they work natively with SSE types with no further work.
			CHECK_EQ(alignment, 16);
			return malloc(size);
#elif defined(__FreeBSD__) || defined(__NetBSD__)
			void *result;

			if (posix_memalign(&result, alignment, size)) {
				// non-zero means allocation error
				// either no allocation or bad alignment value
				return NULL;
			}
			return result;
#else  // This is for Linux.
			return memalign(alignment, size);
#endif
		}

		void aligned_free(void *ptr) {
#ifdef _WIN32
			_aligned_free(ptr);
#else
			free(ptr);
#endif
		}
	}
}  // namespace mvg
