/*******************************************************************************
 * 文件： mvg_macros.h
 * 时间： 2014/11/21 22:52
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 定义一些通用的宏，例如不可复制，简化异常，断言机制等
 *
 ********************************************************************************/
#ifndef MVG_UTILS_MVG_MACROS_H_
#define MVG_UTILS_MVG_MACROS_H_

#include <mvg/base/link_pragmas.h>
#include <sstream> // ostringstream
#include <stdexcept> // logic_error

/**保证这个类不可以复制,可以用utils中的noncopyable.h进行替代*/
#define DISALLOW_COPY_AND_ASSING(TypeName) TypeName(const TypeName&);   void operator=(const TypeName&);

/** 获取当前函数名称的宏，不同的编译器不一样:  */
#if defined(_MSC_VER) && (_MSC_VER>=1300)
#define	__CURRENT_FUNCTION_NAME__	__FUNCTION__
#elif defined(__BORLANDC__)
#define	__CURRENT_FUNCTION_NAME__	__FUNC__
#else
#define	__CURRENT_FUNCTION_NAME__	__PRETTY_FUNCTION__
#endif

/**定义一个统一的方法报告异常
* \param msg 这个可以是一个char*,或者一个std::string.
*/
#define THROW_EXCEPTION(msg)	\
		{\
		std::ostringstream auxCompStr;\
		auxCompStr << "\n\n =============== MVG EXCEPTION =============\n";\
		auxCompStr << __CURRENT_FUNCTION_NAME__ << ", line " << __LINE__ << ":\n";\
		auxCompStr << msg << std::endl; \
		throw std::logic_error( auxCompStr.str() );\
		}\

#define THROW_EXCEPTION_CUSTOM_MSG1(msg,param1)	\
		{\
		std::ostringstream auxCompStr;\
		auxCompStr << "\n\n =============== MVG EXCEPTION =============\n";\
		auxCompStr << __CURRENT_FUNCTION_NAME__ << ", line " << __LINE__ << ":\n";\
		auxCompStr << mvg::format(msg,param1)<< std::endl; \
		throw std::logic_error( auxCompStr.str() );\
		}\

//////////////////////////////////将异常封装成宏，减少代码量的书写////////////////////////////////////////
#	define MVG_TRY_START	\
	try { \

#define THROW_STACKED_EXCEPTION(e)	\
				{\
		std::string str( e.what() );\
		if (str.find("MVG stack trace")==std::string::npos) \
						{ \
			str+= __CURRENT_FUNCTION_NAME__;\
			str+= printf(", line %i:\n", __LINE__ );\
			throw std::logic_error( str );\
						} \
			else throw std::logic_error( e.what() );\
				}\

#	define MVG_TRY_END	\
				} \
	catch (std::bad_alloc &) { throw; } \
	catch (std::exception &e) { THROW_STACKED_EXCEPTION(e); } \
	catch (...) { THROW_EXCEPTION("Unexpected runtime error!"); } \

/**将异常整合成一个宏的形式，另外可以配置一个全局分析，后续添加*/
#define MVG_START  \
	MVG_TRY_START

#define MVG_END  \
	MVG_TRY_END

#define MVG_NO_THROWS		throw()

/** 定义一种断言机制（仅在debug模式下有效）.
* \note 不要把必须执行的语句放入到这个里面，只是做比较，在Release模式下会忽略.
* \sa MVG_TRY_START, MVG_TRY_END
*/
#ifdef _DEBUG
#	define ASSERTDEB_(f) ASSERT_(f)
#	define ASSERTDEBMSG_(f,__ERROR_MSG) ASSERTMSG_(f,__ERROR_MSG)
#else
#	define ASSERTDEB_(f) { }
#	define ASSERTDEBMSG_(f,__ERROR_MSG) { }
#endif


#if MVG_HAS_ASSERT
/**定义一种断言机制.
* \note 不要把必须执行的语句放入，这边只是做一些验证比较.因为在release版本中可能会忽略ASSERT_.* \sa MVG_TRY_START, MVG_TRY_END
*/
#define ASSERTMSG_(f,__ERROR_MSG) { if (!(f)) THROW_EXCEPTION( ::std::string( __ERROR_MSG ) ); }
/** 定义一种断言机制.
* \note 不要把必须执行的语句放入，这边只是做一些验证比较.因为在release版本中可能会忽略ASSERT_.
*/
#define ASSERT_(f) ASSERTMSG_(f, std::string("Assert condition failed: ") + ::std::string(#f) )
#endif

/**	定义内存对齐方式
 */
#if defined(_MSC_VER)
#define MVG_ALIGN16 __declspec(align(16))
#define MVG_ALIGN32 __declspec(align(32))
#else
#define MVG_ALIGN16
#define MVG_ALIGN32
#endif

//定义 M_PI: 依赖于标准库<cmath>
#ifndef M_PI
#	define M_PI 3.14159265358979323846	// PI
#endif

#ifndef M_2PI
#	define M_2PI 6.283185307179586476925286766559	// 2*PI
#endif

#define M_PIf  3.14159265358979f
#define M_2PIf 6.28318530717959f

#if defined(HAVE_LONG_DOUBLE) && !defined(M_PIl)
#	define M_PIl 3.14159265358979323846264338327950288L
#	define M_2PIl (2.0L*3.14159265358979323846264338327950288L)
#endif

namespace mvg
{
	// 在这边再进行申明一下，方便调用
	std::string BASE_IMPEXP format(const char *fmt, ...);
}


#endif // MVG_UTILS_MVG_MACROS_H_
