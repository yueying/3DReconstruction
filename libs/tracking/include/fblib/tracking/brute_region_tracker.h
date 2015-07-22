#ifndef FBLIB_TRACKING_BRUTE_REGION_TRACKER_H_
#define FBLIB_TRACKING_BRUTE_REGION_TRACKER_H_

#include "fblib/image/image.h"
#include "fblib/tracking/region_tracker.h"
using namespace fblib::image;

namespace fblib {
	namespace tracking{
		
		struct BruteRegionTracker : public RegionTracker {
			BruteRegionTracker()
				: half_window_size(4),
				minimum_correlation(0.78) {}

			virtual ~BruteRegionTracker() {}

			/**
			* \brief   实现接口，跟踪一个点从\a image1 到 \a image2
			* 			\a x2, \a y2 为\a image2 中猜测位置，如果没有猜测，则起始点为(\a x1, \a y1)
			*
			* \param	image1	  	第一幅图像
			* \param	image2	  	第二幅图像
			* \param	x1		  	起始点坐标x值
			* \param	y1		  	起始点坐标y值
			* \param [in,out]	x2	可输入猜测值，输出对应跟踪值 对应x坐标
			* \param [in,out]	y2	可输入猜测值，输出对应跟踪值 对应y坐标
			*
			* \return	true if it succeeds, false if it fails.
			*/
			virtual bool Track(const Image<float> &image1,
				const Image<float> &image2,
				double  x1, double  y1,
				double *x2, double *y2) const;

			int half_window_size;
			double minimum_correlation;
		};
	} //namespace tracking
}  // namespace fblib

#endif  // FBLIB_TRACKING_BRUTE_REGION_TRACKER_H_
