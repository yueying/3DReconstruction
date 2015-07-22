/*******************************************************************************
 * 文件： referenced_memory_block.h
 * 时间： 2014/12/08 22:20
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 表示一个共享内存块，用于多线程中信号量的存储
 *
********************************************************************************/
#ifndef FBLIB_UTILS_REFERENCED_MEMORY_BLOCK_H_
#define FBLIB_UTILS_REFERENCED_MEMORY_BLOCK_H_

#include <vector>
#include <memory>

#include <fblib/base/link_pragmas.h>

namespace fblib
{
	namespace utils
	{
		class BASE_IMPEXP ReferencedMemoryBlock : public std::shared_ptr < std::vector<char> >
		{
			typedef std::shared_ptr < std::vector<char> > base_t;

		public:
			/**	构造函数，初始化一个可选的内存块大小
			 */
			ReferencedMemoryBlock(size_t mem_block_size = 0);

			/**	析构函数
			 */
			virtual ~ReferencedMemoryBlock();

			/**	重新设置共享内存块的大小
			 */
			void resize(size_t mem_block_size);

			/**	访问内存块
			 */
			template <class T> T getAs()
			{
				if (!base_t::get())//判断智能指针的裸指针是否为空
					throw std::runtime_error("Trying to access to an uninitialized memory block");

				if (base_t::get()->empty())
					throw std::runtime_error("Trying to access to a memory block of size 0");

				return reinterpret_cast<T>(&base_t::operator ->()->operator [](0));
			}

			/**	访问内存块
			 */
			template <class T> T getAs() const
			{
				if (!base_t::get())
					throw std::runtime_error("Trying to access to an uninitialized memory block");

				if (base_t::get()->empty())
					throw std::runtime_error("Trying to access to a memory block of size 0");

				return reinterpret_cast<const T>(&base_t::operator ->()->operator [](0));
			}
		};
	}
}


#endif // FBLIB_UTILS_REFERENCED_MEMORY_BLOCK_H_