#ifndef FBLIB_BASE_ALIGNED_MALLOC_H_
#define FBLIB_BASE_ALIGNED_MALLOC_H_

namespace fblib {
	namespace utils{
		// Allocate block of size bytes at least aligned to a given value.
		void *aligned_malloc(int size, int alignment);

		// Free memory allocated by aligned_malloc.
		void aligned_free(void *ptr);
	}
}  // namespace fblib

#endif  // FBLIB_BASE_ALIGNED_MALLOC_H_
