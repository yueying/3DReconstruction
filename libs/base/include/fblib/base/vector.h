#ifndef FBLIB_BASE_VECTOR_H
#define FBLIB_BASE_VECTOR_H

#include <cstring>
#include <new>

#include <Eigen/Core>

namespace fblib {
	namespace base{
		
		/**	一个简单的容器类，保证对使用Eigen类型保证16字节对齐  
		 *  不要对那些不可以通过mencpy的类使用
		 *  http://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
		 */
		template <typename T,
			typename Allocator = Eigen::aligned_allocator<T> >
		class vector {
		public:
			~vector()                        { clear(); }

			vector()                         { init(); }
			vector(int size)                 { init(); resize(size); }
			vector(int size, const T & val)  {
				init();
				resize(size);
				std::fill(data_, data_ + size_, val);
			}

			/**	拷贝构造函数 和 赋值
			 */
			vector(const vector<T, Allocator> &rhs) {
				init();
				copy(rhs);
			}
			vector<T, Allocator> &operator=(const vector<T, Allocator> &rhs) {
				if (&rhs != this) {
					copy(rhs);
				}
				return *this;
			}

			/**	交换两个vector中的内容
			 */
			void swap(vector<T, Allocator> &other) {
				std::swap(allocator_, other.allocator_);
				std::swap(size_, other.size_);
				std::swap(capacity_, other.capacity_);
				std::swap(data_, other.data_);
			}

			int      size()            const { return size_; }
			int      capacity()        const { return capacity_; }
			const T& back()            const { return data_[size_ - 1]; }
			T& back()                  { return data_[size_ - 1]; }
			const T& front()           const { return data_[0]; }
			T& front()                 { return data_[0]; }
			const T& operator[](int n) const { return data_[n]; }
			T& operator[](int n)       { return data_[n]; }
			const T * begin()          const { return data_; }
			const T * end()            const { return data_ + size_; }
			T * begin()                { return data_; }
			T * end()                  { return data_ + size_; }

			void resize(int size) {
				reserve(size);
				if (size > size_) {
					construct(size_, size);
				}
				else if (size < size_) {
					destruct(size, size_);
				}
				size_ = size;
			}



			void push_back(const T &value) {
				if (size_ == capacity_) {
					reserve(size_ ? 2 * size_ : 1);
				}
				new (&data_[size_++]) T(value);
			}

			void pop_back() {
				resize(size_ - 1);
			}

			void clear() {
				destruct(0, size_);
				deallocate();
				init();
			}

			void reserve(unsigned int size) {
				if (size > size_) {
					T *data = static_cast<T *>(allocate(size));
					memcpy(data, data_, sizeof(*data)*size_);
					allocator_.deallocate(data_, capacity_);
					data_ = data;
					capacity_ = size;
				}
			}

		private:
			void construct(int start, int end) {
				for (int i = start; i < end; ++i) {
					new (&data_[i]) T;
				}
			}
			void destruct(int start, int end) {
				for (int i = start; i < end; ++i) {
					data_[i].~T();
				}
			}
			void init() {
				size_ = 0;
				data_ = 0;
				capacity_ = 0;
			}

			void *allocate(int size) {
				return size ? allocator_.allocate(size) : 0;
			}

			void deallocate() {
				allocator_.deallocate(data_, size_);
				data_ = 0;
			}

			void copy(const vector<T, Allocator> &rhs) {
				resize(rhs.size());
				for (int i = 0; i < rhs.size(); ++i) {
					(*this)[i] = rhs[i];
				}
			}

			Allocator allocator_;
			unsigned int size_;
			unsigned int capacity_;
			T *data_;
		};
	}
}  // namespace fblib

#endif  // FBLIB_BASE_VECTOR_H
