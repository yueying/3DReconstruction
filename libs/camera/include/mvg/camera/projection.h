#ifndef MVG_CAMERA_PROJECTION_H_
#define MVG_CAMERA_PROJECTION_H_

#include "mvg/camera/link_pragmas.h"
#include "mvg/math/numeric.h"

using namespace mvg::math;

/** \brief	常用的有关投影矩阵相关函数 */
namespace mvg {
	namespace camera{

		/**
		 * \brief	计算投影矩阵 P = K[R|t]
		 *
		 * \param	K		 	相机内参
		 * \param	R		 	相机外参 旋转R
		 * \param	t		 	相机外参 平移t
		 * \param [out]	P	返回的Mat34投影矩阵
		 */
		void CAMERA_IMPEXP P_From_KRt(const Mat3 &K, const Mat3 &R, const Vec3 &t, Mat34 *P);

		/**
		 * \brief	根据投机矩阵P进行分解为相机的内外参，将P进行RQ分解
		 *
		 * \param	P		  	投影矩阵
		 * \param [in,out]	Kp	相机内参
		 * \param [in,out]	Rp	相机外参R
		 * \param [in,out]	tp	相机外参t
		 */
		void CAMERA_IMPEXP KRt_From_P(const Mat34 &P, Mat3 *Kp, Mat3 *Rp, Vec3 *tp);

		/**
		 * \brief	计算3D点离相机的深度即Zc.
		 *
		 * \param	R	相机的外参旋转矩阵
		 * \param	t	相机的外参 平移
		 * \param	X	世界坐标的3D点
		 *
		 * \return	返回相机坐标系下，X点的Zc坐标值
		 */
		double CAMERA_IMPEXP Depth(const Mat3 &R, const Vec3 &t, const Vec3 &X);

		/**
		 * \brief	计算投影后的像素值，P*[X|1.0]
		 *
		 * \param	P	投影矩阵
		 * \param	X	世界坐标系中的点
		 *
		 * \return	图像中的像素坐标
		 */

		Vec2 CAMERA_IMPEXP Project(const Mat34 &P, const Vec3 &X);

		/**
		 * \brief	对一系列三维点计算投影点
		 *
		 * \param	P		 	投影矩阵
		 * \param	X		 	一系列三维点
		 * \param [in,out]	x	计算得到一系列投影点
		 */
		void CAMERA_IMPEXP Project(const Mat34 &P, const Mat3X &X, Mat2X *x);

		/**
		* \brief	对一系列三维点计算投影点
		*
		* \param	P		 	投影矩阵
		* \param	X		 	一系列四维点
		* \param [in,out]	x	计算得到一系列投影点
		*/
		void CAMERA_IMPEXP Project(const Mat34 &P, const Mat4X &X, Mat2X *x);

		/**
		* \brief	对一系列三维点计算投影点
		*
		* \param	P		 	投影矩阵
		* \param	X		 	一系列三维点
		* \return	计算得到一系列投影点
		*/
		Mat2X CAMERA_IMPEXP Project(const Mat34 &P, const Mat3X &X);

		/**
		* \brief	对一系列三维点计算投影点
		*
		* \param	P		 	投影矩阵
		* \param	X		 	一系列四维点
		* \return	计算得到一系列投影点
		*/
		Mat2X CAMERA_IMPEXP Project(const Mat34 &P, const Mat4X &X);

		/**
		* \brief	齐次坐标做笛卡尔坐标
		*
		* \param	H		 	齐次坐标
		* \param [in,out]	X   输出笛卡尔坐标
		*/
		void CAMERA_IMPEXP HomogeneousToEuclidean(const Vec4 &H, Vec3 *X);

		/**
		* \brief	笛卡尔坐标转齐次坐标
		*
		* \param	X		 	笛卡尔坐标
		* \param [in,out]	H   输出齐次坐标
		*/
		void CAMERA_IMPEXP EuclideanToHomogeneous(const Mat &X, Mat *H);

		/**
		* \brief	笛卡尔坐标转齐次坐标
		*
		* \param	X		 	笛卡尔坐标
		* \return   输出齐次坐标
		*/
		Vec3 CAMERA_IMPEXP EuclideanToHomogeneous(const Vec2 &x);

		/**
		* \brief	多个齐次坐标做笛卡尔坐标
		*
		* \param	H		 	齐次坐标
		* \param [in,out]	X   输出笛卡尔坐标
		*/
		void CAMERA_IMPEXP HomogeneousToEuclidean(const Mat &H, Mat *X);

		/**
		* \brief	多个笛卡尔坐标转齐次坐标
		*
		* \param	x		 	笛卡尔坐标
		* \return   输出齐次坐标
		*/
		Mat3X CAMERA_IMPEXP EuclideanToHomogeneous(const Mat2X &x);

		/**
		* \brief	多个笛卡尔坐标转齐次坐标
		*
		* \param	x		 	笛卡尔坐标
		* \param [in,out]	h   输出齐次坐标
		*/
		void CAMERA_IMPEXP EuclideanToHomogeneous(const Mat2X &x, Mat3X *h);

		/**
		* \brief	多个齐次坐标做笛卡尔坐标
		*
		* \param	h		 	齐次坐标
		* \param [in,out]	e   输出笛卡尔坐标
		*/
		void CAMERA_IMPEXP HomogeneousToEuclidean(const Mat3X &h, Mat2X *e);

		/// Project x point in camera coordinates
		void CAMERA_IMPEXP EuclideanToNormalizedCamera(const Mat2X &x, const Mat3 &K, Mat2X *n);

		/// Project x point in camera coordinates
		void CAMERA_IMPEXP HomogeneousToNormalizedCamera(const Mat3X &x, const Mat3 &K, Mat2X *n);

		/// Estimates the root mean square error (2D)
		double CAMERA_IMPEXP RootMeanSquareError(const Mat2X &x_image,
			const Mat4X &X_world,
			const Mat34 &P);

		/// Estimates the root mean square error (2D)
		double CAMERA_IMPEXP RootMeanSquareError(const Mat2X &x_image,
			const Mat3X &X_world,
			const Mat3 &K,
			const Mat3 &R,
			const Vec3 &t);

	}// namespace camera
} // namespace mvg

#endif //MVG_CAMERA_PROJECTION_H_
