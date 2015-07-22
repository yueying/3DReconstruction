#ifndef FBLIB_CAMERA_BROWNPINHOLECAMERA_H
#define FBLIB_CAMERA_BROWNPINHOLECAMERA_H

#include "fblib/math/numeric.h"
#include "fblib/multiview/projection.h"
#include "fblib/camera/pinhole_camera.h"

using namespace fblib::multiview;
namespace fblib{
	namespace camera{
		// Apply camera intrinsics to the normalized point to get image coordinates.
		// This applies the radial lens distortion to a point which is in normalized
		// camera coordinates (i.e. the principal point is at (0, 0)) to get image
		// coordinates in pixels. Templated for use with autodifferentiation.
		template <typename T>
		inline void ApplyRadialDistortionIntrinsics(
			const T &focal_length_x,
			const T &focal_length_y,
			const T &principal_point_x,
			const T &principal_point_y,
			const T &k1,
			const T &k2,
			const T &k3,
			const T &normalized_x,
			const T &normalized_y,
			T *image_x,
			T *image_y) {
			T x = normalized_x;
			T y = normalized_y;

			// Apply distortion to the normalized points to get (xd, yd).
			T r2 = x*x + y*y;
			T r4 = r2 * r2;
			T r6 = r4 * r2;
			T r_coeff = (T(1) + k1*r2 + k2*r4 + k3*r6);
			T xd = x * r_coeff;// + T(2)*p1*x*y + p2*(r2 + T(2)*x*x);
			T yd = y * r_coeff;// + T(2)*p2*x*y + p1*(r2 + T(2)*y*y);

			// Apply focal length and principal point to get the final image coordinates.
			*image_x = principal_point_x + focal_length_x * xd;
			*image_y = principal_point_y + focal_length_y * yd;
		}

		/// Camera : euclidean motion [R|t] and Brown radial decentered distortion model
		struct BrownPinholeCamera
		{
			BrownPinholeCamera(
				double focal = 1,
				double ppx = 0,
				double ppy = 0,
				const Mat3 & R = Mat3::Identity(),
				const Vec3 & t = Vec3::Zero(),
				double k1 = 0.0,
				double k2 = 0.0,
				double k3 = 0.0)
				: rotation_matrix_(R), translation_vector_(t), _f(focal), _ppx(ppx), _ppy(ppy),
				_k1(k1), _k2(k2), _k3(k3)
			{
				camera_center_ = -R.transpose() * t;
				camera_matrix_ << _f, 0, _ppx,
					0, _f, _ppy,
					0, 0, 1;
				P_From_KRt(camera_matrix_, rotation_matrix_, translation_vector_, &projection_matrix_);
			}

			BrownPinholeCamera(const PinholeCamera & projection_matrix) :
				projection_matrix_(projection_matrix.projection_matrix_), rotation_matrix_(projection_matrix.rotation_matrix_), translation_vector_(projection_matrix.translation_vector_), camera_matrix_(projection_matrix.camera_matrix_), _k1(0.0), _k2(0.0), _k3(0.0)
			{
				_f = camera_matrix_(0, 0);
				_ppx = camera_matrix_(0, 2);
				_ppy = camera_matrix_(1, 2);
				camera_center_ = -rotation_matrix_.transpose() * translation_vector_;
			}

			/// Intrinsic parameters (Focal, principal point and radial distortion)
			double _f, _ppx, _ppy, _k1, _k2, _k3;

			///Extrinsic Rotation
			Mat3 rotation_matrix_;

			/// Extrinsic translation
			Vec3 translation_vector_;

			/// Camera center
			Vec3 camera_center_;

			Mat34 projection_matrix_;
			Mat3 camera_matrix_;

			/// Projection of a 3D point into the camera plane (member function)
			Vec2 Project(const Vec3 & world_point) const
			{
				Vec3 X = rotation_matrix_ * world_point + translation_vector_;
				// Point from homogeneous to euclidean.
				double xe = X[0] / X[2];
				double ye = X[1] / X[2];

				double xd, yd;
				ApplyRadialDistortionIntrinsics(
					_f, _f, _ppx, _ppy, _k1, _k2, _k3,
					xe, ye,
					&xd, &yd);

				return Vec2(xd, yd);
			}

			/// Return the residual value to the given 2d point
			double Residual(const Vec3 & world_point, const Vec2 & ref) const  {
				return (ref - Project(world_point)).norm();
			}

			/// Return the squared value of the residual to the givel 2d point
			double ResidualSquared(const Vec3 & world_point, const Vec2 & ref) const  {
				return (ref - Project(world_point)).squaredNorm();
			}

			// Compute the depth of the X point. R*X[2]+t[2].
			double Depth(const Vec3 &X) const{
				return fblib::multiview::Depth(rotation_matrix_, translation_vector_, X);
			}

			/// Return the angle (degree) between two pinhole point rays
			static double AngleBetweenRay(
				const BrownPinholeCamera & cam1,
				const BrownPinholeCamera & cam2,
				const Vec2 & x1, const Vec2 & x2)
			{
				// x = (u, v, 1.0)  // image coordinates
				// X = R.t() * K.inv() * x + C // Camera world point
				// getting the ray:
				// ray = X - C = R.t() * K.inv() * x
				Vec3 ray1 = (cam1.rotation_matrix_.transpose() *
					(cam1.camera_matrix_.inverse() * Vec3(x1(0), x1(1), 1.))).normalized();
				Vec3 ray2 = (cam2.rotation_matrix_.transpose() *
					(cam2.camera_matrix_.inverse() * Vec3(x2(0), x2(1), 1.))).normalized();
				double mag = std::sqrt(ray1.squaredNorm() * ray2.squaredNorm());
				double dotAngle = ray1.dot(ray2);
				return R2D(acos(clamp(dotAngle / mag, -1.0 + 1.e-8, 1.0 - 1.e-8)));
			}

		};
	}
} // namespace fblib

#endif // FBLIB_CAMERA_BROWNPINHOLECAMERA_H

