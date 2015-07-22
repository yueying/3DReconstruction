/*******************************************************************************
 * 文件： notify.h
 * 时间： 2015/04/15 19:24
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 设置消息通知级别
 *
********************************************************************************/
#ifndef FBLIB_UTILS_NOTIFY_H_
#define FBLIB_UTILS_NOTIFY_H_

#include <fblib/base/link_pragmas.h>
#include <fblib/utils/referenced.h> // for NotifyHandler

#include <ostream>

namespace fblib
{
	namespace utils{
		/** 提示等级 可以通过环境变量FBLIBNOTIFYLEVEL 或 FBLIB_NOTIFY_LEVEL进行设置
		  */
		enum NotifySeverity {
			ALWAYS = 0,
			WRONG = 1,
			WARN = 2,
			NOTICE = 3,
			INFO = 4,
			DEBUG_INFO = 5,
			DEBUG_FP = 6
		};

		/** 设置提示等级，这个将覆盖环境变量FBLIBNOTIFYLEVEL 或 FBLIB_NOTIFY_LEVEL 所设置的值.
		  */
		extern BASE_IMPEXP void setNotifyLevel(NotifySeverity severity);

		/** 得到当前消息提示的等级 */
		extern BASE_IMPEXP NotifySeverity getNotifyLevel();

		/** 初始化消息通知级别 */
		extern BASE_IMPEXP bool initNotifyLevel();

		/** 启用消息提示，显示比设置等级大的消息 */
		extern BASE_IMPEXP bool isNotifyEnabled(NotifySeverity severity);


		/** 通过设置NotifyLevel来进行对输出消息进行控制，可以设置环境变量，如：
		  * - set FBLIBNOTIFYLEVEL=DEBUG (for Windows)
		  * 默认等级为NOTICE
		  * 具体使用如下：
		  * @code
		  * fblib::utils::notify(fblib::utils::DEBUG) << "Hello Bugs!" << std::endl;
		  * @endcode
		  * @see setNotifyLevel, setNotifyHandler
		  */
		extern BASE_IMPEXP std::ostream& notify(const NotifySeverity severity);

		inline std::ostream& notify(void) { return notify(fblib::utils::INFO); }

#define FBLIB_NOTIFY(level) if (fblib::utils::isNotifyEnabled(level)) fblib::utils::notify(level)
#define FBLIB_ALWAYS FBLIB_NOTIFY(fblib::utils::ALWAYS)
#define FBLIB_WRONG FBLIB_NOTIFY(fblib::utils::WRONG)
#define FBLIB_WARN FBLIB_NOTIFY(fblib::utils::WARN)
#define FBLIB_NOTICE FBLIB_NOTIFY(fblib::utils::NOTICE)
#define FBLIB_INFO FBLIB_NOTIFY(fblib::utils::INFO)
#define FBLIB_DEBUG FBLIB_NOTIFY(fblib::utils::DEBUG_INFO)
#define FBLIB_DEBUG_FP FBLIB_NOTIFY(fblib::utils::DEBUG_FP)

		/** Handler 处理消息流的输出. 被作为一个输出槽的功能，当消息需要同步时
		  * (如调用 fblib::utils::notify() << std::endl).
		  * StandardNotifyHandler被默认使用，其将消息输出到stderr
		  * (severity <= WARN) 或者 stdout (severity > WARN).
		  * 消息可以被重定向到，其它GUI的窗体上或者windows debugger (WinDebugNotifyHandler).
		  * 使用setNotifyHandle来自定义handler.
		  * 注意：消息输出 API不是线程安全的，输出的GUI的时候要进行线程控制
		  * @see setNotifyHandler
		  */
		class BASE_IMPEXP NotifyHandler : public fblib::utils::Referenced
		{
		public:
			virtual void notify(fblib::utils::NotifySeverity severity, const char *message) = 0;
		};

		/** 设置消息handler, 默认使用StandardNotifyHandler.
		  * @see NotifyHandler
		  */
		extern BASE_IMPEXP void setNotifyHandler(NotifyHandler *handler);

		/** 获取当前的消息通知handler. */
		extern BASE_IMPEXP NotifyHandler *getNotifyHandler();

		/** 将消息通知流重定向到stderr (severity <= WARN) 或者 stdout (severity > WARN).
		  * fputs()函数用于将消息写入到标准文件中，则标准流
		  * std::out and std::cerr streams将不会被使用.
		  * @see setNotifyHandler
		  */
		class BASE_IMPEXP StandardNotifyHandler : public NotifyHandler
		{
		public:
			void notify(fblib::utils::NotifySeverity severity, const char *message);
		};

		/** 将消息通知流重定向到 windows debugger 通过使用 OuputDebugString 函数.
		  * @see setNotifyHandler
		  */
		class BASE_IMPEXP WinDebugNotifyHandler : public NotifyHandler
		{
		public:
			void notify(fblib::utils::NotifySeverity severity, const char *message);
		};

	}
}

#endif // FBLIB_UTILS_NOTIFY_H_
