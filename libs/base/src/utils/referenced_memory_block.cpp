/*******************************************************************************
 * 文件： referenced_memory_block.cpp
 * 时间： 2014/12/09 9:49
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 表示一个共享内存块，用于多线程中信号量的存储
 *
********************************************************************************/
#include "base_precomp.h"
#include <fblib/utils/referenced_memory_block.h>

using namespace fblib::utils;

/**	构造函数
 */
ReferencedMemoryBlock::ReferencedMemoryBlock(size_t mem_block_size) :
base_t(new std::vector<char>(mem_block_size))
{
}

/**	析构函数
 */
ReferencedMemoryBlock::~ReferencedMemoryBlock()
{
}

/**	重新设置共享内存块的大小
 */
void ReferencedMemoryBlock::resize(size_t mem_block_size)
{
	this->operator ->()->resize(mem_block_size);
}