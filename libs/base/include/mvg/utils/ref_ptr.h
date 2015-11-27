#ifndef MVG_UTILS_REF_PTR_H_
#define MVG_UTILS_REF_PTR_H_

namespace mvg
{
	namespace utils{

		template<typename T> class observer_ptr;

		/** 智能指针处理引用计数对象.*/
		template<class T>
		class ref_ptr
		{
		public:
			typedef T element_type;
			/**	默认构造函数
			 */
			ref_ptr() : m_ptr(0) {}
			/**	构造函数，传入参照对象的实例，并将其引用计数加1
			 */
			ref_ptr(T* ptr) : m_ptr(ptr) { if (m_ptr) m_ptr->ref(); }
			ref_ptr(const ref_ptr& rp) : m_ptr(rp.m_ptr) { if (m_ptr) m_ptr->ref(); }
			template<class Other> ref_ptr(const ref_ptr<Other>& rp) : m_ptr(rp.m_ptr) { if (m_ptr) m_ptr->ref(); }
			ref_ptr(observer_ptr<T>& optr) : m_ptr(0) { optr.lock(*this); }
			/**	析构这个智能指针，并将其中参照对象的引用计数减1
			 */
			~ref_ptr() { if (m_ptr) m_ptr->unref();  m_ptr = 0; }

			ref_ptr& operator = (const ref_ptr& rp)
			{
				assign(rp);
				return *this;
			}

			template<class Other> ref_ptr& operator = (const ref_ptr<Other>& rp)
			{
				assign(rp);
				return *this;
			}
			/**	将一个参照对象传入此智能指针，将其引用计数加1，同时删除原来的参照对象，将引用计数减1
			 */
			inline ref_ptr& operator = (T* ptr)
			{
				if (m_ptr == ptr) return *this;
				T* tmp_ptr = m_ptr;
				m_ptr = ptr;
				if (m_ptr) m_ptr->ref();
				// unref second to prevent any deletion of any object which might
				// be referenced by the other object. i.e rp is child of the
				// original m_ptr.
				if (tmp_ptr) tmp_ptr->unref();
				return *this;
			}

			/** 隐式输出转换，将参照对象的实例使用“()”操作符输出
			* 假设ref_pointer 为管理某个参照对象的智能指针变量，则有
			* Referenced* ref = (ref_pointer)
			*/
			operator T*() const { return m_ptr; }
			/**	将参照对象的实例使用“*”操作符输出，即
			 *  Referenced &ref = *ref_pointer
			 */
			T& operator*() const { return *m_ptr; }
			/**	将参照对象的实例使用“->”操作符输出。如：执行Referenced类的func()函数
			 *  ref_pointer->func()
			 */
			T* operator->() const { return m_ptr; }
			/**	将参照对象的实例输出，即：Referenced* ref = ref_pointer.get()
			 */
			T* get() const { return m_ptr; }

			bool operator!() const   { return m_ptr == 0; }
			/**	判断智能指针是否合法，即参照对象是否为空
			 */
			bool valid() const       { return m_ptr != 0; }
			/**	将参照对象的实例输出，同时析构这个智能指针，被输出的参照对象引用计数归0，但是不释放
			 */
			T* release() { T* tmp = m_ptr; if (m_ptr) m_ptr->unref_nodelete(); m_ptr = 0; return tmp; }

			void swap(ref_ptr& rp) { T* tmp = m_ptr; m_ptr = rp.m_ptr; rp.m_ptr = tmp; }

		private:

			template<class Other> void assign(const ref_ptr<Other>& rp)
			{
				if (m_ptr == rp.m_ptr) return;
				T* tmp_ptr = m_ptr;
				m_ptr = rp.m_ptr;
				if (m_ptr) m_ptr->ref();
				// unref second to prevent any deletion of any object which might
				// be referenced by the other object. i.e rp is child of the
				// original m_ptr.
				if (tmp_ptr) tmp_ptr->unref();
			}

			template<class Other> friend class ref_ptr;

			T* m_ptr;
		};


		template<class T> inline
			void swap(ref_ptr<T>& rp1, ref_ptr<T>& rp2) { rp1.swap(rp2); }

		template<class T> inline
			T* get_pointer(const ref_ptr<T>& rp) { return rp.get(); }

		template<class T, class Y> inline
			ref_ptr<T> static_pointer_cast(const ref_ptr<Y>& rp) { return static_cast<T*>(rp.get()); }

		template<class T, class Y> inline
			ref_ptr<T> dynamic_pointer_cast(const ref_ptr<Y>& rp) { return dynamic_cast<T*>(rp.get()); }

		template<class T, class Y> inline
			ref_ptr<T> const_pointer_cast(const ref_ptr<Y>& rp) { return const_cast<T*>(rp.get()); }
	}
}

#endif // MVG_UTILS_REF_PTR_H_
