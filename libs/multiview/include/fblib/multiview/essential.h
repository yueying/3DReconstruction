#ifndef FBLIB_MULTIVIEW_ESSENTIAL_H_
#define FBLIB_MULTIVIEW_ESSENTIAL_H_

#include <vector>
#include "fblib/math/numeric.h"

using namespace fblib::math;

namespace fblib {
	namespace multiview{
		// Compute the relative camera motion between two cameras.
		// Given the motion parameters of two cameras, computes the motion parameters
		// of the second one assuming the first one to be at the origin.
		// If T1 and T2 are the camera motions, the computed relative motion is
		//
		//      T = T2 T1^{-1}
		//
		void RelativeCameraMotion(const Mat3 &R1,
			const Vec3 &t1,
			const Mat3 &R2,
			const Vec3 &t2,
			Mat3 *R,
			Vec3 *t);

		/**
		 * \brief	根据给定的基础矩阵和两相机的内参计算本质矩阵
		 *
		 * \param	fundamental_matrix		 	基础矩阵
		 * \param	K1		 	第一个相机的内参
		 * \param	K2		 	第二个相机的内参
		 * \param [in,out]	E	得到两幅图像之间的本质矩阵
		 */
		void EssentialFromFundamental(const Mat3 &fundamental_matrix,
			const Mat3 &K1,
			const Mat3 &K2,
			Mat3 *E);

		/// Compute E as E = [t12]x R12.
		void EssentialFromRt(const Mat3 &R1,
			const Vec3 &t1,
			const Mat3 &R2,
			const Vec3 &t2,
			Mat3 *E);

		/**
		 * \brief	根据给定的本质矩阵和两相机的内参计算基础矩阵
		 * 			参考：http://ai.stanford.edu/~birch/projective/node20.html
		 *
		 * \param	E		 	两图像之间的本质矩阵
		 * \param	K1		 	第一个相机的内参
		 * \param	K2		 	第二个相机的内参
		 * \param [in,out]	fundamental_matrix	输出两图像之间的基础矩阵
		 */
		void FundamentalFromEssential(const Mat3 &E,
			const Mat3 &K1,
			const Mat3 &K2,
			Mat3 *fundamental_matrix);

		/// Test the possible R|t configuration to have point in front of the cameras
		/// Return false if no possible configuration
		bool MotionFromEssentialAndCorrespondence(const Mat3 &E,
			const Mat3 &K1,
			const Vec2 &x1,
			const Mat3 &K2,
			const Vec2 &x2,
			Mat3 *R,
			Vec3 *t);

		/// Choose one of the four possible motion solutions from an essential matrix.
		/// Decides the right solution by checking that the triangulation of a match
		/// x1--x2 lies in front of the cameras.
		/// Return the index of the right solution or -1 if no solution.
		int MotionFromEssentialChooseSolution(const std::vector<Mat3> &Rs,
			const std::vector<Vec3> &ts,
			const Mat3 &K1,
			const Vec2 &x1,
			const Mat3 &K2,
			const Vec2 &x2);

		/**
		 * \brief	根据本质矩阵恢复相机外参
		 *
		 * \param	E		  	本质矩阵
		 * \param [in,out]	Rs	相机外参R
		 * \param [in,out]	ts	相机外参t
		 */
		void MotionFromEssential(const Mat3 &E,
			std::vector<Mat3> *Rs,
			std::vector<Vec3> *ts);

	}// namespace multiview
} // namespace fblib

#endif // FBLIB_MULTIVIEW_ESSENTIAL_H_
