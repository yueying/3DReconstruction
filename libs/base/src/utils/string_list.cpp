/*******************************************************************************
 * 文件： string_list.cpp
 * 时间： 2014/12/14 0:22
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 定义一个存放字符串列表的类
 *
********************************************************************************/
#include "base_precomp.h"

#include <fblib/utils/string_list.h>
#include <fblib/utils/fblib_macros.h>
#include <fblib/utils/string_utils.h>

using namespace fblib::utils;

/** 默认构造函数，空的字符串列表
*/
StringList::StringList()
{

}

/** 构造函数，输入一个字符串
*/
StringList::StringList(const std::string& text)
{
	FBLIB_START
		setText(text);
	FBLIB_END
}


/** 在字符串列表最后加入一个新的字符串
*/
void  StringList::add(const std::string &str)
{
	m_strings.push_back(str);
}

/** 在给定位置插入新字符串(0表示插入在起始位置，1表示插入第二个位置，...)
*/
void  StringList::insert(size_t index, const std::string &str)
{
	FBLIB_START

	if (index >= m_strings.size()) THROW_EXCEPTION("index out of bounds!");
	m_strings.insert(m_strings.begin() + index, str);

	FBLIB_END
}

/** 对指定位置的值进行重写(0代表起始位置)
*/
void  StringList::set(size_t index, const std::string &str)
{
	FBLIB_START

	if (index >= m_strings.size()) THROW_EXCEPTION("index out of bounds!");
	m_strings[index] = str;

	FBLIB_END
}

/** 清除整个字符串列表
*/
void  StringList::clear()
{
	m_strings.clear();
}


/** 根据给定的位置对字符串进行删除，0是起始位置
*/
void  StringList::remove(size_t index)
{
	FBLIB_START

	if (index >= m_strings.size()) THROW_EXCEPTION("index out of bounds!");
	m_strings.erase(m_strings.begin() + index);

	FBLIB_END
}


/** 查找给定字符串在列表中的位置，找到则返回索引，否则返回false, 默认区分大小
* \return true 字符串找到.
*/
bool  StringList::find(
	const std::string		&compareText,
	size_t					foundIndex,
	bool					caseSensitive) const
{
	FBLIB_START

	foundIndex = 0;
	if (caseSensitive)
	{
		for (std::deque<std::string>::const_iterator it = m_strings.begin(); it != m_strings.end(); ++it, foundIndex++)
			if (!strcmp(compareText.c_str(), it->c_str()))
				return true;
	}
	else
	{
		for (std::deque<std::string>::const_iterator it = m_strings.begin(); it != m_strings.end(); ++it, foundIndex++)
			if (!_strcmpi(compareText.c_str(), it->c_str()))
				return true;
	}

	return false;
	FBLIB_END
}

/** 将整个字符串列表组成一个字符串返回通过'\r\n'.
*/
void  StringList::getText(std::string &outText) const
{
	FBLIB_START
	std::deque<std::string>::const_iterator	it;
	size_t	curPos = 0, totalLen = 0;

	// 1) 计算字符串的总长度，添加'\r\n':
	// ----------------------------------------------------------------
	for (it = m_strings.begin(); it != m_strings.end(); it++)
		totalLen += it->size() + 2;

	outText.resize(totalLen);

	// 2) 将字符串复制输出
	// ----------------------------------------------------------------
	for (it = m_strings.begin(); it != m_strings.end(); it++)
	{
		memcpy_s(&outText[curPos], totalLen, it->c_str(), it->size());
		curPos += it->size();
		outText[curPos++] = '\r';
		outText[curPos++] = '\n';
	}

	FBLIB_END
}

/** 将一个字符串切分通过转义字符'\r', '\n', 或者 '\r\n'.
*/
void  StringList::setText(const std::string &inText)
{
	FBLIB_START
		fblib::utils::tokenize(inText, "\r\n", m_strings);
	FBLIB_END
}


/** 返回字符串列表中字符串个数
*/
size_t  StringList::size() const
{
	return m_strings.size();
}

/** 根据索引返回一个字符串
*/
void  StringList::get(size_t index, std::string &outText) const
{
	FBLIB_START

	if (index >= m_strings.size()) THROW_EXCEPTION("index out of bounds!");
	outText = m_strings[index];

	FBLIB_END
}

/*重载操作符()*/
std::string  StringList::operator ()(size_t index) const
{
	FBLIB_START

	if (index >= m_strings.size()) THROW_EXCEPTION("index out of bounds!");
	return m_strings[index];

	FBLIB_END
}

/*重载操作符()*/
std::string&  StringList::operator ()(size_t index)
{
	FBLIB_START

	if (index >= m_strings.size()) THROW_EXCEPTION("index out of bounds!");
	return m_strings[index];

	FBLIB_END
}

/*根据key返回value*/
std::string  StringList::get_string(const std::string &keyName)
{
	FBLIB_START

	std::string		strToLookFor(keyName + std::string("="));
	size_t			idx = std::string::npos;

	for (std::deque<std::string>::iterator it = m_strings.begin(); it != m_strings.end(); ++it)
	{
		idx = it->find(strToLookFor);
		if (idx == 0)
			return	it->substr(strToLookFor.size());
	}

	THROW_EXCEPTION(format("Key '%s' not found!", keyName.c_str()));

	FBLIB_END
}

/*根据key返回value*/
float StringList::get_float(const std::string &keyName)
{
	FBLIB_START

	std::string	s(get_string(keyName));
	return (float)atof(s.c_str());

	FBLIB_END
}

/*根据key返回value*/
int StringList::get_int(const std::string &keyName)
{
	FBLIB_START

	std::string	s(get_string(keyName));
	return atoi(s.c_str());

	FBLIB_END
}

/*根据key返回value*/
double StringList::get_double(const std::string &keyName)
{
	FBLIB_START

	std::string	s(get_string(keyName));
	return atof(s.c_str());

	FBLIB_END
}

/*根据key返回value*/
bool StringList::get_bool(const std::string &keyName)
{
	FBLIB_START

	std::string	s(get_string(keyName));
	return atoi(s.c_str()) != 0;

	FBLIB_END
}

/*设置值根据给定的keyy ("key=value"),如果存在则覆盖.*/
void  StringList::set(const std::string &keyName, const std::string &value)
{
	FBLIB_START

	std::string		strToLookFor(keyName + std::string("="));
	size_t			idx = std::string::npos;

	for (std::deque<std::string>::iterator it = m_strings.begin(); it != m_strings.end(); ++it)
	{
		idx = it->find(strToLookFor);
		if (idx == 0)
		{
			// 替代已经存在的字符串
			(*it) = strToLookFor + value;
			return;
		}
	}

	// 如果是一个新key添加该字符串
	m_strings.push_back(strToLookFor + value);

	FBLIB_END
}
/*设置值根据给定的keyy ("key=value"),如果存在则覆盖.*/
void  StringList::set(const std::string &keyName, const int &value)
{
	FBLIB_START
		set(keyName, format("%i", value));
	FBLIB_END
}
/*设置值根据给定的keyy ("key=value"),如果存在则覆盖.*/
void  StringList::set(const std::string &keyName, const float &value)
{
	FBLIB_START
		set(keyName, format("%.10e", value));
	FBLIB_END
}

/*设置值根据给定的keyy ("key=value"),如果存在则覆盖.*/
void  StringList::set(const std::string &keyName, const double &value)
{
	FBLIB_START
		set(keyName, format("%.16e", value));
	FBLIB_END
}

/*设置值根据给定的keyy ("key=value"),如果存在则覆盖.*/
void  StringList::set(const std::string &keyName, const bool &value)
{
	FBLIB_START
		set(keyName, std::string(value ? "1" : "0"));
	FBLIB_END
}
