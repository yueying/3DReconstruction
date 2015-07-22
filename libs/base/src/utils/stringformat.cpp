/*******************************************************************************
 * 文件： stringformat.cpp
 * 时间： 2014/11/22 15:31
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 字符串格式化后输出
 *
********************************************************************************/
#include "base_precomp.h"
#include <fblib/utils/stringformat.h>

#include <cstdarg>
#include <vector>

std::string fblib::format(const char *fmt, ...)
{
	if (!fmt) return std::string("");

	int   result = -1, length = 1024;
	std::vector<char> buffer;
	while (result == -1)
	{
		buffer.resize(length + 10);

		va_list args;   //定义保存函数参数的结构
		va_start(args, fmt);//args指向传入的第一个可选参数，fmt是最后一个确定的参数
		result = vsnprintf_s(&buffer[0], length, _TRUNCATE, fmt, args);//如果成功调用此函数，返回写到buffer中的字符的个数
		va_end(args);

		if (result >= length) result = -1;//返回长度溢出
		length *= 2;
	}
	std::string s(&buffer[0]);
	return s;
}