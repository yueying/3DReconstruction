#ifndef MVG_MULTIVIEW_TRIANGULATION_H_
#define MVG_MULTIVIEW_TRIANGULATION_H_

#include "mvg/math/numeric.h"

using namespace mvg::math;

namespace mvg {
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
} // namespace mvg

#endif  // MVG_MULTIVIEW_TRIANGULATION_H_
