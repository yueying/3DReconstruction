#ifndef MVG_CAMERA_PINHOLE_CAMERA_H
#define MVG_CAMERA_PINHOLE_CAMERA_H

#include "mvg/math/numeric.h"
#include "mvg/camera/projection.h"

namespace mvg{
	namespace camera{
		
		/** \brief	给出针孔相机模型 P = K[R|t], t = -RC */
		struct PinholeCamera
		{
			PinholeCamera(
				const Mat3 & K = Mat3::Identity(),
				const Mat3 & R = Mat3::Identity(),
				const Vec3 & t = Vec3::Zero())
				: camera_matrix_(K), rotation_matrix_(R), translation_vector_(t)
			{
				camera_center_ = -R.transpose() * t;
				P_From_KRt(camera_matrix_, rotation_matrix_, translation_vector_, &projection_matrix_);
			}

			PinholeCamera(const Mat34 & projection_matrix)
			{
				projection_matrix_ = projection_matrix;
				KRt_From_P(projection_matrix_, &camera_matrix_, &rotation_matrix_, &translation_vector_);
				camera_center_ = -rotation_matrix_.transpose() * translation_vector_;
			}

			Mat34 projection_matrix_;//!< 投影矩阵 P = K[R|t]

			Mat3 camera_matrix_;//!< 相机矩阵 ，焦距和中心点

			Mat3 rotation_matrix_;//!< 外参 旋转

			Vec3 translation_vector_;//!< 外参平移

			Vec3 camera_center_; //!< 相机光心的世界坐标

			/**
			 * \brief	投影世界坐标系中的3d点到图像平面.
			 *
			 * \param	projection_matrix	相机的投影矩阵
			 * \param	world_point		 	世界坐标系中的点.
			 *
			 * \return	投影到图像平面中的坐标.
			 */

			static Vec2 Project(const Mat34 &projection_matrix, const Vec3 &world_point)
			{
				return mvg::camera::Project(projection_matrix, world_point);
			}

			/**
			 * \brief	投影世界坐标系中的3d点到图像平面
			 *
			 * \param	world_point	世界坐标系中的点
			 *
			 * \return	投影到图像平面中的坐标
			 */
			Vec2 Project(const Vec3 &world_point) const
			{
				return mvg::camera::Project(projection_matrix_, world_point);
			}

			/**
			 * \brief	计算投影与参考值返回的差值.
			 *
			 * \param	projection_matrix	相机的投影矩阵
			 * \param	world_point		 	世界坐标系中的3D点.
			 * \param	ref				 	图像中的参考投影点.
			 *
			 * \return	返回投影误差.
			 */
			static double Residual(
				const Mat34 &projection_matrix,
				const Vec3 &world_point,
				const Vec2 &ref) {
				return (ref - mvg::camera::Project(projection_matrix, world_point)).norm();
			}

			/**
			 * \brief	计算投影与参考值返回的差值.
			 *
			 * \param	world_point	世界坐标系中的3D点
			 * \param	ref		   	图像中的参考投影点
			 *
			 * \return	返回投影误差
			 */
			double Residual(const Vec3 &world_point, const Vec2 &ref) const  {
				return (ref - mvg::camera::Project(projection_matrix_, world_point)).norm();
			}

			double ResidualSquared(const Vec3 &world_point, const Vec2& ref) const  {
				return (ref - mvg::camera::Project(projection_matrix_, world_point)).squaredNorm();
			}

			/**
			* \brief	计算3D点离相机的深度即Zc.
			*
			* \param	X	世界坐标的3D点
			*
			* \return	返回相机坐标系下，X点的Zc坐标值
			*/
			double Depth(const Vec3 &X) const{
				return mvg::camera::Depth(rotation_matrix_, translation_vector_, X);
			}

			/// Return the angle (degree) between two pinhole point rays
			static double AngleBetweenRay(
				const PinholeCamera & cam1,
				const PinholeCamera & cam2,
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
				double mag = ray1.norm() * ray2.norm();
				double dotAngle = ray1.dot(ray2);
				return R2D(acos(clamp(dotAngle / mag, -1.0 + 1.e-8, 1.0 - 1.e-8)));
			}

		};
	} // namespace camera
} // namespace mvg

#endif // MVG_CAMERA_PINHOLE_CAMERA_H

