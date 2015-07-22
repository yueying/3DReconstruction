#ifndef FBLIB_BASE_SCOPED_PTR_H
#define FBLIB_BASE_SCOPED_PTR_H

#include <cassert>
#include <cstddef>

namespace fblib {
	namespace utils{
		/**
		 * A handle for a heap-allocated resource that should be freed when it goes out
		 * of scope. This looks similar to the one found in TR1.
		 */
		template<typename T>
		class scoped_ptr {
		public:
			scoped_ptr(T *resource) : resource_(resource) {}
			~scoped_ptr() { reset(0); }

			T *get()             const { return resource_; }
			T *operator->()      const { return resource_; }
			T &operator*()       const { return *resource_; }

			void reset(T *new_resource) {
				if (sizeof(T)) {
					delete resource_;
				}
				resource_ = new_resource;
			}

			T *release() {
				T *released_resource = resource_;
				resource_ = 0;
				return released_resource;
			}

		private:
			// No copying allowed.
			T *resource_;
		};

		// Same as scoped_ptr but caller must allocate the data
		// with new[] and the destructor will free the memory
		// using delete[].
		template<typename T>
		class scoped_array {
		public:
			scoped_array(T *array) : array_(array) {}
			~scoped_array() { reset(NULL); }

			T *get() const { return array_; }

			T& operator[](std::ptrdiff_t i) const {
				assert(i >= 0);
				assert(array_ != NULL);
				return array_[i];
			}

			void reset(T *new_array) {
				if (sizeof(T)) {
					delete array_;
				}
				array_ = new_array;
			}

			T *release() {
				T *released_array = array_;
				array_ = NULL;
				return released_array;
			}

		private:
			T *array_;

			// Forbid comparison of different scoped_array types.
			template <typename T2> bool operator==(scoped_array<T2> const& p2) const;
			template <typename T2> bool operator!=(scoped_array<T2> const& p2) const;

			// Disallow evil constructors
			scoped_array(const scoped_array&);
			void operator=(const scoped_array&);
		};
	}
}  // namespace fblib

#endif  // FBLIB_BASE_SCOPED_PTR_H
