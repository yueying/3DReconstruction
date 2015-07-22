/*******************************************************************************
 * 文件： string_utils.h
 * 时间： 2014/12/14 1:12
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 字符串工具函数，提供字符串分割，大小写转换，字符编码转换等
 *
********************************************************************************/
#ifndef FBLIB_UTILS_STRING_UTILS_H_
#define FBLIB_UTILS_STRING_UTILS_H_

#include <fblib/base/link_pragmas.h>
#include <fblib/utils/fblib_macros.h>
#include <windows.h>
#include <wchar.h>
#include <deque>
#include <vector>
#include <string>

namespace fblib
{
	namespace utils
	{
		/** 根据给定的分隔符对字符串进行分割
		* 例如:
		* \code
		std::vector<std::string>	tokens;
		tokenize( " - Pepe-Er  Muo"," -",tokens);
		* \endcode
		*
		*  输出:
		*		- "Pepe"
		*		- "Er"
		*		- "Muo"
		*/
		void  BASE_IMPEXP tokenize(
			const std::string			&inString,
			const std::string			&inDelimiters,
			std::deque<std::string>		&outTokens) FBLIB_NO_THROWS;

		/** 根据给定的分隔符对字符串进行分割
		* 例如:
		* \code
		std::vector<std::string>	tokens;
		tokenize( " - Pepe-Er  Muo"," -",tokens);
		* \endcode
		*
		*  输出:
		*		- "Pepe"
		*		- "Er"
		*		- "Muo"
		*/
		void BASE_IMPEXP  tokenize(
			const std::string			&inString,
			const std::string			&inDelimiters,
			std::vector<std::string>		&outTokens) FBLIB_NO_THROWS;
		
		/**	wchar_t 与 string的转换
		 */
		std::string BASE_IMPEXP wchar2string(const wchar_t* pwszSrc);

		/**	按指定字符进行字符串分割
		 */
		bool BASE_IMPEXP split(
			const std::string & src, 
			const std::string& delim, 
			std::vector<std::string>& vec_value);
		
	}
}

#endif // FBLIB_UTILS_STRING_UTILS_H_