#ifndef FBLIB_CAMERA_CAMERA_H_
#define FBLIB_CAMERA_CAMERA_H_

#include "fblib/math/numeric.h"

using namespace fblib::math;

namespace fblib {
	namespace camera{

		/** \brief	相机模型的抽象类 */
		class Camera {
		public:
			virtual ~Camera() {}

			// Return the ray direction of a pixel
			virtual Vec3 Ray(const Vec2f &pixel) = 0;
		};
	}
}  // namespace fblib

#endif  // FBLIB_CAMERA_CAMERA_H_
