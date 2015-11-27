#ifndef FBLIB_MULTIVIEW_CONDITIONING_H_
#define FBLIB_MULTIVIEW_CONDITIONING_H_

#include "fblib/multiview/link_pragmas.h"
#include "fblib/math/numeric.h"

//-- Implementation of normalized coordinates.
// Normalization improve accuracy of results and provide benefits
//  that make scale and coordinate origin invariant.
// The implementation follows Algorithm 4.2 from HZ page 109.
using namespace fblib::math;

namespace fblib {
	namespace multiview{
		// Point conditioning :
		void MULTVIEW_IMPEXP PreconditionerFromPoints(const Mat &points, Mat3 *T);

		/**
		 * \brief	对点进行旋转操作，比如用于图像上的像素点转相机平面上的位置点
		 *
		 * \param	points					  	输入的点
		 * \param	T						  	给出的旋转矩阵
		 * \param [in,out]	transformed_points	输出的点
		 *
		 */
		void MULTVIEW_IMPEXP applyTransformationToPoints(const Mat &points,const Mat3 &T,Mat *transformed_points);

		// Normalize point in [-.5, .5] and return transformation matrix
		void MULTVIEW_IMPEXP NormalizePoints(const Mat &points,
			Mat *normalized_points,
			Mat3 *T);

		/// Point conditioning (compute Transformation matrix)
		void MULTVIEW_IMPEXP PreconditionerFromPoints(int width, int height, Mat3 *T);

		// Point conditioning (isotropic)
		void MULTVIEW_IMPEXP IsotropicPreconditionerFromPoints(const Mat &points, Mat3 *T);

		///  Normalize point rom image coordinates to [-.5, .5]
		void MULTVIEW_IMPEXP NormalizePoints(const Mat &points,
			Mat *normalized_points,
			Mat3 *T, int width, int height);

		void MULTVIEW_IMPEXP NormalizeIsotropicPoints(const Mat &points,
			Mat *normalized_points,
			Mat3 *T);

		/// Unnormalize using Inverse
		struct MULTVIEW_IMPEXP UnnormalizerI {
			// Denormalize the results. See HZ page 109.
			static void Unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H);
		};

		/// Unnormalize using Transpose
		struct MULTVIEW_IMPEXP UnnormalizerT {
			// Denormalize the results. See HZ page 109.
			static void Unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H);
		};
	}// namespace multiview
} //namespace fblib

#endif // FBLIB_MULTIVIEW_CONDITIONING_H_
