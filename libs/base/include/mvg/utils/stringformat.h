/*******************************************************************************
 * 文件： stringformat.h
 * 时间： 2014/11/22 15:22
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 定义一个类似sprintf的方法，不过不仅仅是格式化还可以返回格式化后的字符串
 *
********************************************************************************/
#ifndef MVG_UTILS_STRINGFORMAT_H_
#define MVG_UTILS_STRINGFORMAT_H_

#include <mvg/base/link_pragmas.h>
#include <string>

namespace mvg
{
	/**格式化字符串后输出*/
	std::string BASE_IMPEXP format(const char *fmt, ...);
}

#endif // MVG_UTILS_STRINGFORMAT_H_
