/*******************************************************************************
 * 文件： do_opencv_includes.h
 * 时间： 2015/01/04 10:07
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： OpenCV包含,考虑了OpenCV不同的版本，底层图像转换通过IplImage
 *
********************************************************************************/
#ifndef FBLIB_DO_OPENCV_INCL_H
#define FBLIB_DO_OPENCV_INCL_H

#include <fblib/config.h>

#if FBLIB_HAS_OPENCV

#	if FBLIB_OPENCV_VERSION_NUM>=0x211
#	if !defined(__cplusplus)
#		include <opencv2/core/core_c.h>
#		include <opencv2/highgui/highgui_c.h>
#		include <opencv2/imgproc/imgproc_c.h>
#	else
#		include <opencv2/core/core.hpp>
#		include <opencv2/core/core_c.h>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/highgui/highgui_c.h>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#		include <opencv2/features2d/features2d.hpp>
#		include <opencv2/video/tracking.hpp>
#		if FBLIB_OPENCV_VERSION_NUM>=0x300
#			include <opencv2/video/tracking_c.h>
#		endif
#		include <opencv2/calib3d/calib3d.hpp>
#		include <opencv2/objdetect/objdetect.hpp>

#		include <opencv2/legacy/legacy.hpp>  // CvImage
#		include <opencv2/legacy/compat.hpp>
#		if (FBLIB_OPENCV_VERSION_NUM>=0x240) && FBLIB_HAS_OPENCV_NONFREE
#			include <opencv2/nonfree/nonfree.hpp>
#		endif
#	endif
#	else
		// For OpenCV <=2.1
#		include <cv.h>
#		include <highgui.h>
#		include <cvaux.h>
#	endif


#endif // FBLIB_DO_OPENCV_INCL_H

#endif
