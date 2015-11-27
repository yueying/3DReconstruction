/*******************************************************************************
 * 文件： datetime.cpp
 * 时间： 2014/11/19 21:37
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 构建一个系统独立的时间/日期函数
 *
 ********************************************************************************/

#include "base_precomp.h"  // 预编译头


#include <mvg/system/datetime.h>
#include <mvg/utils/mvg_macros.h>

#include <conio.h>
#include <windows.h>
#include <process.h>
#include <tlhelp32.h>
#include <sys/utime.h>
#include <io.h>
#include <direct.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <mvg/utils/stringformat.h>

using namespace mvg;
using namespace mvg::system;
using namespace std;

/*---------------------------------------------------------------
					time_tToTimestamp
---------------------------------------------------------------*/
mvg::system::TTimeStamp  mvg::system::time_tToTimestamp(const time_t &t)
{
	return (((uint64_t)t) * (uint64_t)10000000) + ((uint64_t)116444736 * 1000000000);
}

/*---------------------------------------------------------------
					time_tToTimestamp
---------------------------------------------------------------*/
mvg::system::TTimeStamp  mvg::system::time_tToTimestamp(const double &t)
{
	return (uint64_t)(t*10000000.0) + ((uint64_t)116444736 * 1000000000);
}

/*---------------------------------------------------------------
					timestampTotime_t
---------------------------------------------------------------*/
double mvg::system::timestampTotime_t(const mvg::system::TTimeStamp  &t)
{
	return double(t - ((uint64_t)116444736 * 1000000000)) / 10000000.0;
}


/*---------------------------------------------------------------
				返回当前系统时间
---------------------------------------------------------------*/
mvg::system::TTimeStamp  mvg::system::getCurrentTime()
{
	FILETIME		t;
	GetSystemTimeAsFileTime(&t);
	return (((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime);

}

/*---------------------------------------------------------------
					timestampToParts
---------------------------------------------------------------*/
void mvg::system::timestampToParts(TTimeStamp t, TTimeParts &p, bool localTime)
{
	double T = mvg::system::timestampTotime_t(t);
	time_t tt = time_t(T);

	double sec_frac = T - tt;
	ASSERT_(sec_frac < 1.0);

	struct tm parts;
	int err = 0;
	if (localTime)
	{
		err = localtime_s(&parts, &tt);
	} 
	else
	{
		err=  gmtime_s(&parts,&tt);
	}
	
	ASSERTMSG_(err != 0, "Malformed timestamp");

	p.year = parts.tm_year + 1900;
	p.month = parts.tm_mon + 1;
	p.day = parts.tm_mday;
	p.day_of_week = parts.tm_wday + 1;
	p.daylight_saving = parts.tm_isdst;
	p.hour = parts.tm_hour;
	p.minute = parts.tm_min;
	p.second = parts.tm_sec + sec_frac;
}



/*---------------------------------------------------------------
					buildTimestampFromParts
---------------------------------------------------------------*/
TTimeStamp mvg::system::buildTimestampFromParts(const TTimeParts &p)
{
	struct tm parts;

	parts.tm_year = p.year - 1900;
	parts.tm_mon = p.month - 1;
	parts.tm_mday = p.day;
	parts.tm_wday = p.day_of_week - 1;
	parts.tm_isdst = p.daylight_saving;
	parts.tm_hour = p.hour;
	parts.tm_min = p.minute;
	parts.tm_sec = int(p.second);

	double sec_frac = p.second - parts.tm_sec;

	time_t  tt = _mkgmtime(&parts); 

	return mvg::system::time_tToTimestamp(double(tt) + sec_frac);
}

/*---------------------------------------------------------------
					buildTimestampFromPartsLocalTime
---------------------------------------------------------------*/
TTimeStamp mvg::system::buildTimestampFromPartsLocalTime(const TTimeParts &p)
{
	struct tm parts;

	parts.tm_year = p.year - 1900;
	parts.tm_mon = p.month - 1;
	parts.tm_mday = p.day;
	parts.tm_wday = p.day_of_week - 1;
	parts.tm_isdst = p.daylight_saving;
	parts.tm_hour = p.hour;
	parts.tm_min = p.minute;
	parts.tm_sec = int(p.second);

	double sec_frac = p.second - parts.tm_sec;

	time_t  tt = mktime(&parts);

	return mvg::system::time_tToTimestamp(double(tt) + sec_frac);
}

/*---------------------------------------------------------------
					Returns the current local time.
---------------------------------------------------------------*/
mvg::system::TTimeStamp  mvg::system::getCurrentLocalTime()
{
	FILETIME		tt,t;
	GetSystemTimeAsFileTime(&tt);
	FileTimeToLocalFileTime(&tt,&t);
	return (((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime);
}

/*---------------------------------------------------------------
					timeDifference
---------------------------------------------------------------*/
double mvg::system::timeDifference(const mvg::system::TTimeStamp &t1, const mvg::system::TTimeStamp &t2)
{
	MVG_START
		ASSERT_(t1 != INVALID_TIMESTAMP)
		ASSERT_(t2 != INVALID_TIMESTAMP)

		return ((double)((int64_t)(t2 - t1))) / 10000000.0;

	MVG_END
}

/*---------------------------------------------------------------
					secondsToTimestamp
---------------------------------------------------------------*/
mvg::system::TTimeStamp mvg::system::secondsToTimestamp(const double &nSeconds)
{
	return (mvg::system::TTimeStamp)(nSeconds*10000000.0);
}

/*---------------------------------------------------------------
					formatTimeInterval
---------------------------------------------------------------*/
string mvg::system::formatTimeInterval(const double &t)
{
	double timeSeconds = (t < 0) ? (-t) : t;

	unsigned int nHours = (unsigned int)timeSeconds / 3600;
	unsigned int nMins = ((unsigned int)timeSeconds % 3600) / 60;
	unsigned int nSecs = (unsigned int)timeSeconds % 60;
	unsigned int milSecs = (unsigned int)(1000 * (timeSeconds - floor(timeSeconds)));

	return format(
		"%02u:%02u:%02u.%03u",
		nHours,
		nMins,
		nSecs,
		milSecs);
}

/*---------------------------------------------------------------
  将一个时间戳转变为一个文本形式(UTC time): YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string  mvg::system::dateTimeToString(const mvg::system::TTimeStamp &t)
{
	if (t == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736 * 1000000000));
	time_t          auxTime = tmp / (uint64_t)10000000;
	unsigned int	secFractions = (unsigned int)(1000000 * (tmp % 10000000) / 10000000.0);
	tm  ptm;
	if (gmtime_s(&ptm, &auxTime))
		return std::string("(Malformed timestamp)");

	return format(
		"%u/%02u/%02u,%02u:%02u:%02u.%06u",
		1900 + ptm.tm_year,
		ptm.tm_mon + 1,
		ptm.tm_mday,
		ptm.tm_hour,
		ptm.tm_min,
		(unsigned int)ptm.tm_sec,
		secFractions);
}

/*---------------------------------------------------------------
  将一个时间戳转变为一个文本形式(local time): YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
string  mvg::system::dateTimeLocalToString(const mvg::system::TTimeStamp &t)
{
	if (t == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736 * 1000000000));
	time_t          auxTime = tmp / (uint64_t)10000000;
	unsigned int	secFractions = (unsigned int)(1000000 * (tmp % 10000000) / 10000000.0);
	tm  ptm;

	if (localtime_s(&ptm, &auxTime)) 
		return "(Malformed timestamp)";

	return format(
		"%u/%02u/%02u,%02u:%02u:%02u.%06u",
		1900 + ptm.tm_year,
		ptm.tm_mon + 1,
		ptm.tm_mday,
		ptm.tm_hour,
		ptm.tm_min,
		(unsigned int)ptm.tm_sec,
		secFractions);
}

/*---------------------------------------------------------------
						根据时间戳返回一个与当天零点差值
---------------------------------------------------------------*/
double  mvg::system::extractDayTimeFromTimestamp(const mvg::system::TTimeStamp &t)
{
	MVG_START
		ASSERT_(t != INVALID_TIMESTAMP)

	SYSTEMTIME		sysT;
	FileTimeToSystemTime( (FILETIME*)&t, &sysT );
	return sysT.wHour * 3600.0 + sysT.wMinute * 60.0 + sysT.wSecond + sysT.wMilliseconds * 0.001;

	MVG_END
}


/*---------------------------------------------------------------
  将一个时间戳转变为时间文本形式(local time): HH:MM:SS.MMMMMM
  ---------------------------------------------------------------*/
string  mvg::system::timeLocalToString(const mvg::system::TTimeStamp &t, unsigned int secondFractionDigits)
{
	if (t == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736 * 1000000000));
	time_t          auxTime = tmp / (uint64_t)10000000;
	unsigned int	secFractions = (unsigned int)(1000000 * (tmp % 10000000) / 10000000.0);
	tm  ptm;
	if (localtime_s(&ptm, &auxTime))
		return "(Malformed timestamp)";

	return format(
		"%02u:%02u:%02u.%0*u",
		ptm.tm_hour,
		ptm.tm_min,
		(unsigned int)ptm.tm_sec,
		secondFractionDigits,
		secFractions);
}

/*---------------------------------------------------------------
  将一个时间戳转变为时间文本形式(UTC): HH:MM:SS.MMMMMM
  ---------------------------------------------------------------*/
string  mvg::system::timeToString(const mvg::system::TTimeStamp &t)
{
	if (t == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736 * 1000000000));
	time_t          auxTime = tmp / (uint64_t)10000000;
	unsigned int	secFractions = (unsigned int)(1000000 * (tmp % 10000000) / 10000000.0);
	tm  ptm;

	if (gmtime_s(&ptm, &auxTime))
		return string("(Malformed timestamp)");

	return format(
		"%02u:%02u:%02u.%06u",
		ptm.tm_hour,
		ptm.tm_min,
		(unsigned int)ptm.tm_sec,
		secFractions);
}

/*---------------------------------------------------------------
  将一个时间戳转变为文本形式，只有日期: YEAR/MONTH/DAY
  ---------------------------------------------------------------*/
string  mvg::system::dateToString(const mvg::system::TTimeStamp &t)
{
	if (t == INVALID_TIMESTAMP) return string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736 * 1000000000));
	time_t          auxTime = tmp / (uint64_t)10000000;
	tm  ptm;
	if (gmtime_s(&ptm, &auxTime))
		return string("(Malformed timestamp)");

	return format(
		"%u/%02u/%02u",
		1900 + ptm.tm_year,
		ptm.tm_mon + 1,
		ptm.tm_mday
		);
}

/** 这个函数实现时间的格式化:例如给出一个以秒为单位的时间,它将返回一个合适的描述根据不同的时间尺度.
 * 例如: 1.23 年, 3.50 天, 9.3 小时, 5.3 分钟, 3.34 秒, 178.1 毫秒,  87.1 纳秒.
 * \sa unitsFormat
 */
std::string mvg::system::intervalFormat(const double seconds)
{
	if (seconds >= 365 * 24 * 3600)
		return format("%.2f years", seconds / (365 * 24 * 3600));
	else if (seconds >= 24 * 3600)
		return format("%.2f days", seconds / (24 * 3600));
	else if (seconds >= 3600)
		return format("%.2f hours", seconds / 3600);
	else if (seconds >= 60)
		return format("%.2f minutes", seconds / 60);
	else if (seconds >= 1)
		return format("%.2f sec", seconds);
	else if (seconds >= 1e-3)
		return format("%.2f ms", seconds*1e3);
	else if (seconds >= 1e-6)
		return format("%.2f us", seconds*1e6);
	else	return format("%.2f ns", seconds*1e9);
}
