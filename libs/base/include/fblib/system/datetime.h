/*******************************************************************************
 * 文件： datetime.h
 * 时间： 2014/11/19 21:28
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 时间和日期函数，后期可以统一进行平台独立
 *
********************************************************************************/
#ifndef  FBLIB_SYSTEM_DATETIME_H_
#define  FBLIB_SYSTEM_DATETIME_H_

#include <fblib/base/link_pragmas.h>
#include <fblib/utils/fblib_stdint.h>    // 编译器独立的"stdint.h"版本

#include <string>

/** 用这个宏代表一个无效的时间戳. */
#define INVALID_TIMESTAMP (0)

namespace fblib
{
	namespace system
	{
		/** @defgroup time_date 时间和日期函数(in #include <fblib/system/datetime.h>)
		  * \ingroup fblib_base_grp
		  * @{ */

		/** 系统独立的时间类型, 表示的是100毫微秒相对于1601年1月1号(UTC)的间隔.其实也就是unsigned long long类型
		 * \sa system::getCurrentTime, system::timeDifference, INVALID_TIMESTAMP, TTimeParts
		 */
		typedef uint64_t TTimeStamp;

		/** 时间/日期分开的结构体，即将时间表示为年月日时秒分，星期.
		 * \sa TTimeStamp, timestampToParts, buildTimestampFromParts
		 */
		struct TTimeParts
		{
			uint16_t	year;	/** 年 */
			uint8_t		month;  /** 月 (1-12) */
			uint8_t		day;    /** 日 (1-31) */
			uint8_t		hour;   /** 小时 (0-23) */
			uint8_t		minute; /** 分钟 (0-59) */
			double		second; /** 秒 (0.0000-59.9999) */
			uint8_t		day_of_week; /** 星期 (1:Sunday, 7:Saturday) */
			int			daylight_saving;/**日照时间*/
		};

		/** 将分开表示的时间（相对于世界标准时间）表示为一个时间戳
		  * \sa timestampToParts
		  */
		fblib::system::TTimeStamp BASE_IMPEXP buildTimestampFromParts( const fblib::system::TTimeParts &p );

		/** 将分开表示的时间（相对于当地时间）表示为一个时间戳
		  * \sa timestampToParts, buildTimestampFromParts
		  */
		fblib::system::TTimeStamp BASE_IMPEXP buildTimestampFromPartsLocalTime( const fblib::system::TTimeParts &p );

		/** 将时间戳转变为 (日期,时，分，秒)
		  * \sa buildTimestampFromParts
		  */
		void BASE_IMPEXP timestampToParts( TTimeStamp t, TTimeParts &p, bool localTime = false );

		/** 返回当前时间(UTC).
		  * \sa now,getCurrentLocalTime
		  */
		fblib::system::TTimeStamp  BASE_IMPEXP getCurrentTime( );

		/** 一个快捷方式system::getCurrentTime
		  * \sa getCurrentTime, getCurrentLocalTime
		 */
		inline fblib::system::TTimeStamp now() {
			return getCurrentTime();
		}

		/** 返回当前当地时间
		  * \sa now,getCurrentTime
		  */
		fblib::system::TTimeStamp  BASE_IMPEXP getCurrentLocalTime( );

		/** 将标准库中的"time_t"转换成TTimeStamp(实际上也就是转变为一个double类型的值).
		  * \sa timestampTotime_t
		  */
		fblib::system::TTimeStamp  BASE_IMPEXP time_tToTimestamp( const double &t );

		/** 将标准库中"time_t" 转变为 TTimeStamp.
		  * \sa timestampTotime_t
		  */
		fblib::system::TTimeStamp  BASE_IMPEXP time_tToTimestamp( const time_t &t );

		/** 将TTimeStamp转变为标准的"time_t".
		  * \sa time_tToTimestamp, secondsToTimestamp
		  */
		double BASE_IMPEXP timestampTotime_t( const fblib::system::TTimeStamp  &t );

		/** 将TTimeStamp转变为标准的"time_t"，就是一个内联函数更形象的表示，因为time_t就是long long类型，用double表示.
		  * \sa time_tToTimestamp, secondsToTimestamp
		  */
		inline double timestampToDouble( const fblib::system::TTimeStamp  &t ) { return timestampTotime_t(t); }

		/** 返回时间差t1到t2 (如果t2大于t1，则反过来),单位秒.
		  * \sa secondsToTimestamp
		  */
		double BASE_IMPEXP timeDifference( const fblib::system::TTimeStamp &t_first, const fblib::system::TTimeStamp &t_later );

		/** 转变一个时间间隔（单位秒）到TTimeStamp (例如.可以添加到一个已经存在的有效的时间戳)
		  * \sa timeDifference
		  */
		fblib::system::TTimeStamp BASE_IMPEXP secondsToTimestamp( const double &nSeconds );

		/** 对给出的时间差进行格式化, 转变为类似字符串 [H]H:MM:SS.MILISECS
		  * \sa unitsFormat
		  */
		std::string BASE_IMPEXP formatTimeInterval( const double &timeSeconds );

		/** 将一个时间戳转变为一个文本形式(UTC time): YEAR/MONTH/DAY,HH:MM:SS.MMM
		  * \sa dateTimeLocalToString
		  */
		std::string  BASE_IMPEXP dateTimeToString(const fblib::system::TTimeStamp &t);

		/** 将一个时间戳转变为一个文本形式(local time): YEAR/MONTH/DAY,HH:MM:SS.MMM
		  * \sa dateTimeToString
		  */
		std::string  BASE_IMPEXP dateTimeLocalToString(const fblib::system::TTimeStamp &t);

		/** 将一个时间戳转变为文本形式，只有日期: YEAR/MONTH/DAY
		 */
		std::string  BASE_IMPEXP dateToString(const fblib::system::TTimeStamp &t);

		/** 根据时间戳返回一个与当天零点差值
		 */
		double  BASE_IMPEXP extractDayTimeFromTimestamp(const fblib::system::TTimeStamp &t);

		/** 将一个时间戳转变为时间文本形式(UTC): HH:MM:SS.MMMMMM
		 */
		std::string  BASE_IMPEXP timeToString(const fblib::system::TTimeStamp &t);

		/** 将一个时间戳转变为时间文本形式(local time): HH:MM:SS.MMMMMM
		 */
		std::string  BASE_IMPEXP timeLocalToString(const fblib::system::TTimeStamp &t, unsigned int secondFractionDigits=6);

		/** 这个函数实现时间的格式化:例如给出一个以秒为单位的时间,它将返回一个合适的描述根据不同的时间尺度.
		 * 例如: 1.23 年, 3.50 天, 9.3 小时, 5.3 分钟, 3.34 秒, 178.1 毫秒,  87.1 纳秒.
		 * \sa unitsFormat
		 */
		std::string BASE_IMPEXP intervalFormat(const double seconds);

		/** @} */

	} // End of namespace

} // End of namespace

#endif //FBLIB_SYSTEM_DATETIME_H_
