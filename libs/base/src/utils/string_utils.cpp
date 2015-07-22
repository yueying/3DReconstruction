/*******************************************************************************
 * 文件： string_utils.cpp
 * 时间： 2014/12/14 1:19
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 字符串工具函数，提供字符串分割
 *
 ********************************************************************************/
#include "base_precomp.h"

#include <fblib/utils/string_utils.h>
#include <fblib/synch/critical_section.h>

namespace fblib
{
	namespace utils
	{
		void  tokenize(const std::string	&inString, const std::string	&inDelimiters,
			std::deque<std::string>	&outTokens) FBLIB_NO_THROWS
		{
			static fblib::synch::CriticalSection cs;
			fblib::synch::CriticalSectionLocker lock(&cs);

			char	*nextTok, *context;

			outTokens.clear();

			char *dupStr = ::_strdup(inString.c_str());

			nextTok = strtok_s(dupStr, inDelimiters.c_str(), &context);
			while (nextTok != NULL)
			{
				outTokens.push_back(std::string(nextTok));
				nextTok = strtok_s(NULL, inDelimiters.c_str(), &context);
			}

			free(dupStr);
		}


			void  tokenize(const std::string &inString, const std::string &inDelimiters,
			std::vector<std::string>	&outTokens) FBLIB_NO_THROWS
		{
			static fblib::synch::CriticalSection cs;
			fblib::synch::CriticalSectionLocker lock(&cs);


			char	*nextTok, *context;

			outTokens.clear();
			char *dupStr = ::_strdup(inString.c_str());

			nextTok = strtok_s(dupStr, inDelimiters.c_str(), &context);
			while (nextTok != NULL)
			{
				outTokens.push_back(std::string(nextTok));
				nextTok = strtok_s(NULL, inDelimiters.c_str(), &context);
			};

			free(dupStr);
		}


		std::string wchar2string(const wchar_t* pwszSrc)
		{
			int nLen = WideCharToMultiByte(CP_ACP, 0, pwszSrc, -1, NULL, 0, NULL, NULL);
			if (nLen <= 0) return std::string("");

			char* pszDst = new char[nLen];
			if (NULL == pszDst) return std::string("");

			WideCharToMultiByte(CP_ACP, 0, pwszSrc, -1, pszDst, nLen, NULL, NULL);
			pszDst[nLen - 1] = 0;

			std::string strTemp(pszDst);
			delete[] pszDst;

			return strTemp;
		}
		bool split(const std::string & src, const std::string& delim, std::vector<std::string>& vec_value)
		{
			bool bDelimiterExist = false;
			if (!delim.empty())
			{
				vec_value.clear();
				std::string::size_type start = 0;
				std::string::size_type end = std::string::npos - 1;
				while (end != std::string::npos)
				{
					end = src.find(delim, start);
					vec_value.push_back(src.substr(start, end - start));
					start = end + delim.size();
				}
				if (vec_value.size() >= 2)
					bDelimiterExist = true;
			}
			return bDelimiterExist;
		}
		
	}
}
