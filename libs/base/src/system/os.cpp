/*******************************************************************************
 * 文件： os.cpp
 * 时间： 2014/11/22 12:48
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 提供平台独立的一些有用的函数，包括文件操作，时间日期处理，字符串解析，文件I/O，多线程，内存分配等
 *
********************************************************************************/
#include "base_precomp.h"  // 预编译头

#include <mvg/system/os.h>
#include <windows.h>

unsigned int mvg::system::getNumberOfProcessors()
{
	static unsigned int ret = 0;

	if (!ret)
	{
		SYSTEM_INFO si;  
		GetSystemInfo(&si);
		ret = si.dwNumberOfProcessors;
		if (ret < 1) ret = 1;
	}
	return ret;
}

/**	格式化输出
*/
int mvg::system::sprintf(char *buf, size_t bufSize, const char *format, ...) MVG_NO_THROWS
{
	int			result;
	va_list		ap;
	va_start(ap, format);
	result = ::vsprintf_s(buf, bufSize, format, ap);
	va_end(ap);
	return result;
}
