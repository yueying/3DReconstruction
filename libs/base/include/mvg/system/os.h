/*******************************************************************************
 * 文件： os.h
 * 时间： 2014/11/22 12:30
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 提供平台独立的一些有用的函数，包括文件操作，时间日期处理，字符串解析，文件I/O，多线程，内存分配等
 *        对一些不跨平台的函数提出，为后期可能需要跨平台做基础,
 * TODO： 目前只是将可能需要区分处理的使用量较大的基础函数提出，并目前只是加上了windows的处理，后期有需要可以将其他平台的处理加入
 *
********************************************************************************/
#ifndef MVG_SYSTEM_OS_H_
#define MVG_SYSTEM_OS_H_

#include <mvg/config.h>
#include <mvg/base/link_pragmas.h> 
#include <mvg/utils/mvg_macros.h>

namespace mvg
{
	namespace system
	{	
		/**	返回处理器的核心数
		 */
		unsigned int BASE_IMPEXP getNumberOfProcessors();

		/**	格式化输出
		 */
		int BASE_IMPEXP sprintf(char *buf, size_t bufSize, const char *format, ...) MVG_NO_THROWS;

	} // End of namespace

} // End of namespace


#endif // MVG_SYSTEM_OS_H_