#ifndef FBLIB_MULTIVIEW_ROTATION_AVERAGING_COMMON_H_
#define FBLIB_MULTIVIEW_ROTATION_AVERAGING_COMMON_H_

#include "fblib/math/numeric.h"
using namespace fblib::math;

namespace fblib   {
	namespace multiview  {

		// 相对旋转数据的表示形式
		struct RelRotationData {
			size_t i, j; // 两个视图的索引
			Mat3 Rij; // 视图的相对旋转关系
			float weight;

			RelRotationData(size_t i_ = 0, size_t j_ = 0, const	Mat3 & Rij_ = Mat3::Identity(), float weight_ = 1.0f) :
				i(i_), j(j_), Rij(Rij_), weight(weight_)
			{}
		};

	} // namespace multiview
} // namespace fblib

#endif //FBLIB_MULTIVIEW_ROTATION_AVERAGING_COMMON_H_

