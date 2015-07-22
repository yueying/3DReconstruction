#ifndef FBLIB_MULTIVIEW_TRIANGULATION_H_
#define FBLIB_MULTIVIEW_TRIANGULATION_H_

#include "fblib/math/numeric.h"

using namespace fblib::math;

namespace fblib {
	namespace multiview{
		/**	直接线性变换算法，给出的是齐次解
		 */
		void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
			const Mat34 &P2, const Vec2 &x2,
			Vec4 *X_homogeneous);

		/**	直线线性变换算法，给出的是欧拉解
		 */
		void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
			const Mat34 &P2, const Vec2 &x2,
			Vec3 *X_euclidean);

	}// namespace multiview
} // namespace fblib

#endif  // FBLIB_MULTIVIEW_TRIANGULATION_H_
