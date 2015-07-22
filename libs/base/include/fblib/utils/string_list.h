/*******************************************************************************
 * 文件： string_list.h
 * 时间： 2014/12/14 0:04
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 定义一个存放字符串列表的类
 *
********************************************************************************/
#ifndef FBLIB_UTILS_STRING_LIST_H_
#define FBLIB_UTILS_STRING_LIST_H_

#include <fblib/base/link_pragmas.h>
#include <deque>
#include <iterator>
#include <vector>

namespace fblib
{
	namespace utils
	{
		class BASE_IMPEXP StringList
		{
		protected:
			/** 字符串列表
			*/
			std::deque<std::string>		m_strings;

		public:
			/** 默认构造函数，空的字符串列表
			*/
			StringList();

			/** 构造函数，输入一个字符串
			*/
			StringList(const std::string& text);

			/** 显示构造函数 输入deque<string> */
			explicit StringList(const std::deque<std::string>& lines) : m_strings(lines) { }

			/** 显示构造函数 输入vector<string> */
			explicit StringList(const std::vector<std::string>& lines)
			{
				std::copy(lines.begin(), lines.end(), std::back_inserter(m_strings));
			}

			/** 在字符串列表最后加入一个新的字符串
			*/
			void  add(const std::string &str);

			/** 重载<<，用来添加新字符串*/
			StringList & operator << (const std::string &s) { add(s); return *this; }

			/** 在给定位置插入新字符串(0表示插入在起始位置，1表示插入第二个位置，...)
			*/
			void  insert(size_t index, const std::string &str);

			/** 对指定位置的值进行重写(0代表起始位置)
			*/
			void  set(size_t index, const std::string &str);

			/** 清除整个字符串列表
			*/
			void  clear();

			/** 返回字符串列表中字符串个数
			*/
			size_t  size() const;

			/** 根据给定的位置对字符串进行删除，0是起始位置
			*/
			void  remove(size_t index);

			/** 查找给定字符串在列表中的位置，找到则返回索引，否则返回false, 默认区分大小
			* \return true 字符串找到.
			*/
			bool  find(
				const std::string		&compareText,
				size_t					foundIndex,
				bool					caseSensitive = true) const;

			/** 根据索引返回一个字符串
			*/
			void  get(size_t index, std::string &outText) const;

			/** 重载()返回一个字符串
			*/
			std::string  operator ()(size_t index) const;

			/** 重载()返回一个字符串的引用
			*/
			std::string&  operator ()(size_t index);

			/** 将整个字符串列表组成一个字符串返回通过'\r\n'.
			*/
			void  getText(std::string &outText) const;

			/** 将整个字符串列表组成一个字符串返回通过'\r\n'.
			*/
			inline std::string getText() const
			{
				std::string s;
				getText(s);
				return s;
			}

			/** 将一个字符串切分通过转义字符'\r', '\n', 或者 '\r\n'.
			*/
			void  setText(const std::string &inText);

			/** 根据key返回value
			*/
			std::string  get_string(const std::string &keyName);

			/** 根据key返回value
			*/
			float  get_float(const std::string &keyName);

			/** 根据key返回value
			*/
			int  get_int(const std::string &keyName);

			/** 根据key返回value
			*/
			double  get_double(const std::string &keyName);

			/**根据key返回value
			*/
			bool  get_bool(const std::string &keyName);

			/** 设置值根据给定的keyy ("key=value"),如果存在则覆盖.
			*/
			void  set(const std::string &keyName, const std::string &value);

			/** 设置值根据给定的keyy ("key=value"),如果存在则覆盖.
			*/
			void  set(const std::string &keyName, const int &value);

			/** 设置值根据给定的keyy ("key=value"),如果存在则覆盖.
			*/
			void  set(const std::string &keyName, const float &value);

			/** 设置值根据给定的keyy ("key=value"),如果存在则覆盖.
			*/
			void  set(const std::string &keyName, const double &value);

			/** 设置值根据给定的keyy ("key=value"),如果存在则覆盖.
			*/
			void  set(const std::string &keyName, const bool &value);
		};
	}
}

#endif // FBLIB_UTILS_STRING_LIST_H_