/*******************************************************************************
* 文件： safe_pointers.h
* 时间： 2014/11/19 14:56
* 作者： 冯兵
* 邮件： fengbing123@gmail.com
*
* 说明： 对指针进行封装，是=等操作具有安全性
*
********************************************************************************/
#ifndef FBLIB_UTILS_SAFE_POINTERS_H_
#define FBLIB_UTILS_SAFE_POINTERS_H_

namespace fblib
{
	namespace utils
	{
		//TODO:有机会了解一下指针=安全性问题
		/** 对指针的封装，这样对于=操作就没有任何安全性问题.
		* 这个类不需要对指针进行计数，会自动的释放指针的值.
		*/
		template <class T>
		struct safe_ptr_basic
		{
		protected:
			T *ptr;

		public:
			safe_ptr_basic() : ptr(NULL) { }
			safe_ptr_basic(const safe_ptr_basic<T> &o) : ptr(o.ptr) { }
			safe_ptr_basic(const T* p) : ptr(const_cast<T*>(p)) { }
			safe_ptr_basic<T> &operator =(T * p) { ptr = p; return *this; }

			safe_ptr_basic<T> &operator =(const safe_ptr_basic<T>&o)
			{
				ptr = o.ptr;
				return *this;
			}

			virtual ~safe_ptr_basic() {  }

			bool operator == (const T *o) const { return o == ptr; }
			bool operator == (const safe_ptr_basic<T> &o)const { return o.ptr == ptr; }

			bool operator != (const T *o)const { return o != ptr; }
			bool operator != (const safe_ptr_basic<T> &o)const { return o.ptr != ptr; }

			T*& get() { return ptr; }
			const T* get()const { return ptr; }

			T *& operator ->() { ASSERT_(ptr); return ptr; }
			const T * operator ->() const  { ASSERT_(ptr); return ptr; }
		};

		/** 对指针的封装类，这样可以安全的复制通过"="操作符而不会带来任何问题.
		* 这个类不要保持任何的指针计数，会自动释放指向的值.
		*/
		template <class T>
		struct safe_ptr : safe_ptr_basic < T >
		{
		public:
			safe_ptr() : safe_ptr_basic<T>() { }
			safe_ptr(const safe_ptr<T> &o) : safe_ptr_basic<T>(o) { }
			safe_ptr(const T* p) : safe_ptr_basic<T>(p) { }

			virtual ~safe_ptr() { }

			T & operator *() { ASSERT_(safe_ptr_basic<T>::ptr); return *safe_ptr_basic<T>::ptr; }
			const T & operator *() const  { ASSERT_(safe_ptr_basic<T>::ptr); return *safe_ptr_basic<T>::ptr; }

			T & operator [](const size_t &i) { ASSERT_(safe_ptr_basic<T>::ptr); return safe_ptr_basic<T>::ptr[i]; }
			const T & operator [](const size_t &i) const { ASSERT_(safe_ptr_basic<T>::ptr); return safe_ptr_basic<T>::ptr[i]; }
		};

		template <class T>
		struct non_copiable_ptr_basic
		{
		protected:
			T *ptr;

		public:
			non_copiable_ptr_basic() : ptr(NULL) { }
			non_copiable_ptr_basic(const non_copiable_ptr_basic<T> &) : ptr(NULL) { THROW_EXCEPTION("Pointer non-copiable..."); }
			non_copiable_ptr_basic(const T* p) : ptr(const_cast<T*>(p)) { }
			non_copiable_ptr_basic<T> &operator =(T * p) { ptr = p; return *this; }

			non_copiable_ptr_basic<T> &operator =(const non_copiable_ptr_basic<T>&)
			{
				THROW_EXCEPTION("Pointer non-copiable...");
			}

			/** This method can change the pointer, since the change is made explicitly, not through copy operators transparent to the user. */
			void set(const T* p) { ptr = const_cast<T*>(p); }

			virtual ~non_copiable_ptr_basic() {  }

			bool operator == (const T *o) const { return o == ptr; }
			bool operator == (const non_copiable_ptr_basic<T> &o)const { return o.ptr == ptr; }

			bool operator != (const T *o)const { return o != ptr; }
			bool operator != (const non_copiable_ptr_basic<T> &o)const { return o.ptr != ptr; }

			T*& get() { return ptr; }
			const T* get()const { return ptr; }

			T** getPtrToPtr() { return &ptr; }

			T *& operator ->() { ASSERT_(ptr); return ptr; }
			const T * operator ->() const  { ASSERT_(ptr); return ptr; }
		};

		typedef safe_ptr_basic<void> void_ptr;
		typedef non_copiable_ptr_basic<void> void_ptr_noncopy;
	}
}

#endif // FBLIB_UTILS_SAFE_POINTERS_H_