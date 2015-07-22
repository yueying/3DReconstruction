/*******************************************************************************
 * 文件： notify.cpp
 * 时间： 2015/04/15 19:24
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 设置消息通知级别
 *
********************************************************************************/
#include "base_precomp.h"
#include <fblib/utils/notify.h>
#include <fblib/utils/ref_ptr.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iostream>

#include <ctype.h>

#define FBLIB_INIT_SINGLETON_PROXY(ProxyName, Func) static struct ProxyName{ ProxyName() { Func; } } s_##ProxyName;

namespace fblib
{
	namespace utils{

		class NullStreamBuffer : public std::streambuf
		{
		private:
			std::streamsize xsputn(const std::streambuf::char_type * /*str*/, std::streamsize n)
			{
				return n;
			}
		};

		struct NullStream : public std::ostream
		{
		public:
			NullStream() :
				std::ostream(new NullStreamBuffer)
			{
				m_buffer = dynamic_cast<NullStreamBuffer *>(rdbuf());
			}

			~NullStream()
			{
				rdbuf(0);
				delete m_buffer;
			}

		protected:
			NullStreamBuffer* m_buffer;
		};

		/** 当缓冲区需要同步时，流缓冲区调用通知处理程序(通常在 std::endl).
		 */
		struct NotifyStreamBuffer : public std::stringbuf
		{
			NotifyStreamBuffer() : m_severity(fblib::utils::NOTICE)
			{
			}

			void setNotifyHandler(fblib::utils::NotifyHandler *handler) { m_handler = handler; }
			fblib::utils::NotifyHandler *getNotifyHandler() const { return m_handler.get(); }

			/** 设置消息通知等级，给下一次通知使用 */
			void setCurrentSeverity(fblib::utils::NotifySeverity severity)
			{
				if (m_severity != severity)
				{
					sync();
					m_severity = severity;
				}
			}

			fblib::utils::NotifySeverity getCurrentSeverity() const { return m_severity; }

		private:

			int sync()
			{
				sputc(0); // 字符串终止
				if (m_handler.valid())
					m_handler->notify(m_severity, pbase());
				pubseekpos(0, std::ios_base::out); // or str(std::string())
				return 0;
			}

			fblib::utils::ref_ptr<fblib::utils::NotifyHandler> m_handler;
			fblib::utils::NotifySeverity m_severity;
		};

		struct NotifyStream : public std::ostream
		{
		public:
			NotifyStream() :
				std::ostream(new NotifyStreamBuffer)
			{
				m_buffer = dynamic_cast<NotifyStreamBuffer *>(rdbuf());
			}

			void setCurrentSeverity(fblib::utils::NotifySeverity severity)
			{
				m_buffer->setCurrentSeverity(severity);
			}

			fblib::utils::NotifySeverity getCurrentSeverity() const
			{
				return m_buffer->getCurrentSeverity();
			}

			~NotifyStream()
			{
				rdbuf(0);
				delete m_buffer;
			}

		protected:
			NotifyStreamBuffer* m_buffer;
		};

	}
}

using namespace fblib::utils;


struct NotifySingleton
{
    NotifySingleton()
    {
        //默认消息等级
        m_notifyLevel = fblib::utils::NOTICE;

        char* FBLIBNOTIFYLEVEL=getenv("FBLIB_NOTIFY_LEVEL");
        if (!FBLIBNOTIFYLEVEL) FBLIBNOTIFYLEVEL=getenv("FBLIBNOTIFYLEVEL");
        if(FBLIBNOTIFYLEVEL)
        {
            std::string stringFBLIBNOTIFYLEVEL(FBLIBNOTIFYLEVEL);

            // 转换为大写
            for(std::string::iterator i=stringFBLIBNOTIFYLEVEL.begin();
                i!=stringFBLIBNOTIFYLEVEL.end();
                ++i)
            {
                *i=toupper(*i);
            }

            if(stringFBLIBNOTIFYLEVEL.find("ALWAYS")!=std::string::npos)          m_notifyLevel=fblib::utils::ALWAYS;
            else if(stringFBLIBNOTIFYLEVEL.find("WRONG")!=std::string::npos)      m_notifyLevel=fblib::utils::WRONG;
            else if(stringFBLIBNOTIFYLEVEL.find("WARN")!=std::string::npos)       m_notifyLevel=fblib::utils::WARN;
            else if(stringFBLIBNOTIFYLEVEL.find("NOTICE")!=std::string::npos)     m_notifyLevel=fblib::utils::NOTICE;
            else if(stringFBLIBNOTIFYLEVEL.find("DEBUG_INFO")!=std::string::npos) m_notifyLevel=fblib::utils::DEBUG_INFO;
            else if(stringFBLIBNOTIFYLEVEL.find("DEBUG_FP")!=std::string::npos)   m_notifyLevel=fblib::utils::DEBUG_FP;
            else if(stringFBLIBNOTIFYLEVEL.find("DEBUG")!=std::string::npos)      m_notifyLevel=fblib::utils::DEBUG_INFO;
            else if(stringFBLIBNOTIFYLEVEL.find("INFO")!=std::string::npos)       m_notifyLevel=fblib::utils::INFO;
            else std::cout << "Warning: invalid FBLIB_NOTIFY_LEVEL set ("<<stringFBLIBNOTIFYLEVEL<<")"<<std::endl;

        }

        // 设置标准的消息通知handler
        fblib::utils::NotifyStreamBuffer *buffer = dynamic_cast<fblib::utils::NotifyStreamBuffer *>(m_notifyStream.rdbuf());
        if (buffer && !buffer->getNotifyHandler())
            buffer->setNotifyHandler(new StandardNotifyHandler);
    }

    fblib::utils::NotifySeverity m_notifyLevel;
    fblib::utils::NullStream     m_nullStream;
    fblib::utils::NotifyStream   m_notifyStream;
};

static NotifySingleton& getNotifySingleton()
{
    static NotifySingleton s_NotifySingleton;
    return s_NotifySingleton;
}

bool fblib::utils::initNotifyLevel()
{
    getNotifySingleton();
    return true;
}

// 通过代理模式，强制NotifySingleton初始化
FBLIB_INIT_SINGLETON_PROXY(NotifySingletonProxy, fblib::utils::initNotifyLevel())

void fblib::utils::setNotifyLevel(fblib::utils::NotifySeverity severity)
{
    getNotifySingleton().m_notifyLevel = severity;
}

fblib::utils::NotifySeverity fblib::utils::getNotifyLevel()
{
    return getNotifySingleton().m_notifyLevel;
}

void fblib::utils::setNotifyHandler(fblib::utils::NotifyHandler *handler)
{
    fblib::utils::NotifyStreamBuffer *buffer = static_cast<fblib::utils::NotifyStreamBuffer*>(getNotifySingleton().m_notifyStream.rdbuf());
    if (buffer) buffer->setNotifyHandler(handler);
}

fblib::utils::NotifyHandler* fblib::utils::getNotifyHandler()
{
    fblib::utils::NotifyStreamBuffer *buffer = static_cast<fblib::utils::NotifyStreamBuffer *>(getNotifySingleton().m_notifyStream.rdbuf());
    return buffer ? buffer->getNotifyHandler() : 0;
}


bool fblib::utils::isNotifyEnabled( fblib::utils::NotifySeverity severity )
{
    return severity<=getNotifySingleton().m_notifyLevel;
}

std::ostream& fblib::utils::notify(const fblib::utils::NotifySeverity severity)
{
    if (fblib::utils::isNotifyEnabled(severity))
    {
        getNotifySingleton().m_notifyStream.setCurrentSeverity(severity);
        return getNotifySingleton().m_notifyStream;
    }
    return getNotifySingleton().m_nullStream;
}

void fblib::utils::StandardNotifyHandler::notify(fblib::utils::NotifySeverity severity, const char *message)
{
    if (severity <= fblib::utils::WARN)
        fputs(message, stderr);
    else
        fputs(message, stdout);
}

#if defined(WIN32) && !defined(__CYGWIN__)

#ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

void fblib::utils::WinDebugNotifyHandler::notify(fblib::utils::NotifySeverity severity, const char *message)
{
    OutputDebugStringA(message);
}

#endif