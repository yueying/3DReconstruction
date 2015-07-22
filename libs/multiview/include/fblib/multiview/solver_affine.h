#ifndef FBLIB_SOLVER_MULTIVIEW_AFFINE_H_
#define FBLIB_SOLVER_MULTIVIEW_AFFINE_H_

#include "fblib/math/numeric.h"
using namespace fblib::math;

namespace fblib {
	namespace multiview{
		/** 2D 仿射变换估计
		 *  主要用于估计两个已知2D坐标的的序列点之间的仿射变换，具体如下:
		 *      |a b tx|
		 *  M = |c d ty|
		 *      |0 0 1 |
		 *  x2 = M * x1
		 *  对于上述问题最终转化为，已知参数
		 *  a b x
		 *  c d y
		 *  0 0 1
		 *
		 * 解线性方程组 A x = B :
		 * | X1 Y1 0  0  1 0 |  | a |   | X2 |
		 * | 0  0  X1 Y1 0 1 |  | b | = | Y2 |
		 *         ...          | c |     ..
		 *                      | d |
		 *                      | x |
		 *                      | y |
		 *
		 * \param[in] x1 第一个2xN包含欧式坐标点的矩阵
		 * \param[in] x2 第二个2xN包含欧式坐标点的矩阵
		 * \param[out] M 3x3仿射变换矩阵(6自由度)
		 *          
		 * \param[in] expected_precision 预期精度，用于适配几乎所有的仿射变换实例
		 *
		 * \return true 如果仿射变换矩阵估值成果
		 *
		 * \note 需要3个非共线的点
		 */
		bool Affine2DFromCorrespondencesLinear(const Mat &x1,
			const Mat &x2,
			Mat3 *M,
			double expected_precision =
			EigenDoubleTraits::dummy_precision());

		
		/** 3D 仿射变换估计
		*  主要用于估计两个已知3D坐标的的序列点之间的仿射变换，具体如下:
		*              |a b c tx|
		*          M = |d e f ty|
		*              |g h i tz|
		*              |0 0 0 1 |
		*  x2 = M * x1
		*  对于上述问题最终转化为，已知参数
		*  a b c x
		*  d e f y
		*  g h i z
		*  0 0 0 1
		*
		* 解线性方程组 A x = B :
		* | X1 Y1 Z1 0  0  0  0  0  0  1 0 0|  | a |   | X2 |
		* | 0  0  0  X1 Y1 Z1 0  0  0  0 1 0|  | b | = | Y2 |
		* | 0  0  0  0  0  0  X1 Y1 Z1 0 0 1|  | c |   | Z2 |
		*                   ...                | d |     ..
		*                                      | e |
		*                                      | f |
		*                                      | g |
		*                                      | h |
		*                                      | i |
		*                                      | x |
		*                                      | y |
		*                                      | z |
		*
		* \param[in] x1 第一个3xN包含欧式坐标点的矩阵
		* \param[in] x2 第二个3xN包含欧式坐标点的矩阵
		* \param[out] M 4x4仿射变换矩阵(12自由度)
		*
		* \param[in] expected_precision 预期精度，用于适配几乎所有的仿射变换实例
		*
		* \return true 如果仿射变换矩阵估值成果
		*
		* \note 需要4个非共面的点
		*/
		bool Affine3DFromCorrespondencesLinear(const Mat &x1,
			const Mat &x2,
			Mat4 *M,
			double expected_precision =
			EigenDoubleTraits::dummy_precision());

	}// namespace multiview
} // namespace fblib

#endif  // FBLIB_SOLVER_MULTIVIEW_AFFINE_H_
