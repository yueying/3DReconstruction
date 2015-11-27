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
#include <mvg/utils/notify.h>
#include <mvg/utils/ref_ptr.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iostream>

#include <ctype.h>

#define MVG_INIT_SINGLETON_PROXY(ProxyName, Func) static struct ProxyName{ ProxyName() { Func; } } s_##ProxyName;

namespace mvg
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
			NotifyStreamBuffer() : m_severity(mvg::utils::NOTICE)
			{
			}

			void setNotifyHandler(mvg::utils::NotifyHandler *handler) { m_handler = handler; }
			mvg::utils::NotifyHandler *getNotifyHandler() const { return m_handler.get(); }

			/** 设置消息通知等级，给下一次通知使用 */
			void setCurrentSeverity(mvg::utils::NotifySeverity severity)
			{
				if (m_severity != severity)
				{
					sync();
					m_severity = severity;
				}
			}

			mvg::utils::NotifySeverity getCurrentSeverity() const { return m_severity; }

		private:

			int sync()
			{
				sputc(0); // 字符串终止
				if (m_handler.valid())
					m_handler->notify(m_severity, pbase());
				pubseekpos(0, std::ios_base::out); // or str(std::string())
				return 0;
			}

			mvg::utils::ref_ptr<mvg::utils::NotifyHandler> m_handler;
			mvg::utils::NotifySeverity m_severity;
		};

		struct NotifyStream : public std::ostream
		{
		public:
			NotifyStream() :
				std::ostream(new NotifyStreamBuffer)
			{
				m_buffer = dynamic_cast<NotifyStreamBuffer *>(rdbuf());
			}

			void setCurrentSeverity(mvg::utils::NotifySeverity severity)
			{
				m_buffer->setCurrentSeverity(severity);
			}

			mvg::utils::NotifySeverity getCurrentSeverity() const
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

using namespace mvg::utils;


struct NotifySingleton
{
    NotifySingleton()
    {
        //默认消息等级
        m_notifyLevel = mvg::utils::NOTICE;

        char* MVGNOTIFYLEVEL=getenv("MVG_NOTIFY_LEVEL");
        if (!MVGNOTIFYLEVEL) MVGNOTIFYLEVEL=getenv("MVGNOTIFYLEVEL");
        if(MVGNOTIFYLEVEL)
        {
            std::string stringMVGNOTIFYLEVEL(MVGNOTIFYLEVEL);

            // 转换为大写
            for(std::string::iterator i=stringMVGNOTIFYLEVEL.begin();
                i!=stringMVGNOTIFYLEVEL.end();
                ++i)
            {
                *i=toupper(*i);
            }

            if(stringMVGNOTIFYLEVEL.find("ALWAYS")!=std::string::npos)          m_notifyLevel=mvg::utils::ALWAYS;
            else if(stringMVGNOTIFYLEVEL.find("WRONG")!=std::string::npos)      m_notifyLevel=mvg::utils::WRONG;
            else if(stringMVGNOTIFYLEVEL.find("WARN")!=std::string::npos)       m_notifyLevel=mvg::utils::WARN;
            else if(stringMVGNOTIFYLEVEL.find("NOTICE")!=std::string::npos)     m_notifyLevel=mvg::utils::NOTICE;
            else if(stringMVGNOTIFYLEVEL.find("DEBUG_INFO")!=std::string::npos) m_notifyLevel=mvg::utils::DEBUG_INFO;
            else if(stringMVGNOTIFYLEVEL.find("DEBUG_FP")!=std::string::npos)   m_notifyLevel=mvg::utils::DEBUG_FP;
            else if(stringMVGNOTIFYLEVEL.find("DEBUG")!=std::string::npos)      m_notifyLevel=mvg::utils::DEBUG_INFO;
            else if(stringMVGNOTIFYLEVEL.find("INFO")!=std::string::npos)       m_notifyLevel=mvg::utils::INFO;
            else std::cout << "Warning: invalid MVG_NOTIFY_LEVEL set ("<<stringMVGNOTIFYLEVEL<<")"<<std::endl;

        }

        // 设置标准的消息通知handler
        mvg::utils::NotifyStreamBuffer *buffer = dynamic_cast<mvg::utils::NotifyStreamBuffer *>(m_notifyStream.rdbuf());
        if (buffer && !buffer->getNotifyHandler())
            buffer->setNotifyHandler(new StandardNotifyHandler);
    }

    mvg::utils::NotifySeverity m_notifyLevel;
    mvg::utils::NullStream     m_nullStream;
    mvg::utils::NotifyStream   m_notifyStream;
};

static NotifySingleton& getNotifySingleton()
{
    static NotifySingleton s_NotifySingleton;
    return s_NotifySingleton;
}

bool mvg::utils::initNotifyLevel()
{
    getNotifySingleton();
    return true;
}

// 通过代理模式，强制NotifySingleton初始化
MVG_INIT_SINGLETON_PROXY(NotifySingletonProxy, mvg::utils::initNotifyLevel())

void mvg::utils::setNotifyLevel(mvg::utils::NotifySeverity severity)
{
    getNotifySingleton().m_notifyLevel = severity;
}

mvg::utils::NotifySeverity mvg::utils::getNotifyLevel()
{
    return getNotifySingleton().m_notifyLevel;
}

void mvg::utils::setNotifyHandler(mvg::utils::NotifyHandler *handler)
{
    mvg::utils::NotifyStreamBuffer *buffer = static_cast<mvg::utils::NotifyStreamBuffer*>(getNotifySingleton().m_notifyStream.rdbuf());
    if (buffer) buffer->setNotifyHandler(handler);
}

mvg::utils::NotifyHandler* mvg::utils::getNotifyHandler()
{
    mvg::utils::NotifyStreamBuffer *buffer = static_cast<mvg::utils::NotifyStreamBuffer *>(getNotifySingleton().m_notifyStream.rdbuf());
    return buffer ? buffer->getNotifyHandler() : 0;
}


bool mvg::utils::isNotifyEnabled( mvg::utils::NotifySeverity severity )
{
    return severity<=getNotifySingleton().m_notifyLevel;
}

std::ostream& mvg::utils::notify(const mvg::utils::NotifySeverity severity)
{
    if (mvg::utils::isNotifyEnabled(severity))
    {
        getNotifySingleton().m_notifyStream.setCurrentSeverity(severity);
        return getNotifySingleton().m_notifyStream;
    }
    return getNotifySingleton().m_nullStream;
}

void mvg::utils::StandardNotifyHandler::notify(mvg::utils::NotifySeverity severity, const char *message)
{
    if (severity <= mvg::utils::WARN)
        fputs(message, stderr);
    else
        fputs(message, stdout);
}

#if defined(WIN32) && !defined(__CYGWIN__)

#ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

void mvg::utils::WinDebugNotifyHandler::notify(mvg::utils::NotifySeverity severity, const char *message)
{
    OutputDebugStringA(message);
}

#endif