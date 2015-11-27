#ifndef MVG_BASE_ALIGNED_MALLOC_H_
#define MVG_BASE_ALIGNED_MALLOC_H_

namespace mvg {
	namespace utils{
		// Allocate block of size bytes at least aligned to a given value.
		void *aligned_malloc(int size, int alignment);

		// Free memory allocated by aligned_malloc.
		void aligned_free(void *ptr);
	}
}  // namespace mvg

#endif  // MVG_BASE_ALIGNED_MALLOC_H_
