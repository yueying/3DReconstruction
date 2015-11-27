#ifndef MVG_CAMERA_CAMERA_H_
#define MVG_CAMERA_CAMERA_H_

#include "mvg/math/numeric.h"

using namespace mvg::math;

namespace mvg {
	namespace camera{

		/** \brief	相机模型的抽象类 */
		class Camera {
		public:
			virtual ~Camera() {}

			// Return the ray direction of a pixel
			virtual Vec3 Ray(const Vec2f &pixel) = 0;
		};
	}
}  // namespace mvg

#endif  // MVG_CAMERA_CAMERA_H_
