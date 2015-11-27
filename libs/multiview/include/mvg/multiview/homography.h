#ifndef MVG_MULTIVIEW_HOMOGRAPHY_H_
#define MVG_MULTIVIEW_HOMOGRAPHY_H_

#include "mvg/math/numeric.h"
using namespace mvg::math;

namespace mvg {
	namespace multiview{

		/**
		* This structure contains options that controls how the homography
		* estimation operates.
		*
		* Defaults should be suitable for a wide range of use cases, but
		* better performance and accuracy might require tweaking/
		*/
		struct EstimateHomographyOptions {
			// Default constructor which sets up a options for generic usage.
			EstimateHomographyOptions(void);

			// Normalize correspondencies before estimating the homography
			// in order to increase estimation stability.
			//
			// Normaliztion will make it so centroid od correspondences
			// is the coordinate origin and their average distance from
			// the origin is sqrt(2).
			//
			// See:
			//   - R. Hartley and A. Zisserman. Multiple View Geometry in Computer
			//     Vision. Cambridge University Press, second edition, 2003.
			//   - https://www.cs.ubc.ca/grads/resources/thesis/May09/Dubrofsky_Elan.pdf
			bool use_normalization;

			// Maximal number of iterations for the refinement step.
			int max_num_iterations;

			// Expected average of symmetric geometric distance between
			// actual destination points and original ones transformed by
			// estimated homography matrix.
			//
			// Refinement will finish as soon as average of symmetric
			// geometric distance is less or equal to this value.
			//
			// This distance is measured in the same units as input points are.
			double expected_average_symmetric_distance;
		};

		/**
		 * 2D homography transformation estimation.
		 *
		 * This function estimates the homography transformation from a list of 2D
		 * correspondences which represents either:
		 *
		 * - 3D points on a plane, with a general moving camera.
		 * - 3D points with a rotating camera (pure rotation).
		 * - 3D points + different planar projections
		 *
		 * \param x1 The first 2xN or 3xN matrix of euclidean or homogeneous points.
		 * \param x2 The second 2xN or 3xN matrix of euclidean or homogeneous points.
		 * \param  H The 3x3 homography transformation matrix (8 dof) such that
		 *               x2 = H * x1   with       |a b c|
		 *                                    H = |d e f|
		 *                                        |g h 1|
		 * \param expected_precision The expected precision in order for instance
		 *                           to accept almost homography matrices.
		 *
		 * \return True if the transformation estimation has succeeded.
		 * \note There must be at least 4 non-colinear points.
		 */
		bool Homography2DFromCorrespondencesLinear(const Mat &x1,
			const Mat &x2,
			Mat3 *H,
			double expected_precision =
			EigenDouble::dummy_precision());

		

		/**
		 * 2D homography transformation estimation.
		 *
		 * This function estimates the homography transformation from a list of 2D
		 * correspondences by doing algebraic estimation first followed with result
		 * refinement.
		 */
		bool EstimateHomography2DFromCorrespondences(
			const Mat &x1,
			const Mat &x2,
			const EstimateHomographyOptions &options,
			Mat3 *H);

		/**
		 * 3D 单应矩阵估计
		 *
		 * This function can be used in order to estimate the homography transformation
		 * from a list of 3D correspondences.
		 *
		 * \param[in] x1 The first 4xN matrix of homogeneous points
		 * \param[in] x2 The second 4xN matrix of homogeneous points
		 * \param[out] H The 4x4 homography transformation matrix (15 dof) such that
		 *               x2 = H * x1   with       |a b c d|
		 *                                    H = |e f g h|
		 *                                        |i j k l|
		 *                                        |m n o 1|
		 * \param[in] expected_precision The expected precision in order for instance
		 *        to accept almost homography matrices.
		 *
		 * \return true if the transformation estimation has succeeded
		 *
		 * \note Need at least 5 non coplanar points
		 * \note Points coordinates must be in homogeneous coordinates
		 */
		bool Homography3DFromCorrespondencesLinear(const Mat &x1,
			const Mat &x2,
			Mat4 *H,
			double expected_precision =
			EigenDouble::dummy_precision());

		/**
		 * \brief 计算对称转移误差，同时考虑前向变换(H)和后向变换(H^-1)
		 * 并把这两种变换对应的几何误差累计，所得误差为：
		 *  D(H * x1, x2)^2 + D(H^-1 * x2, x1)
		 */
		double SymmetricGeometricDistance(const Mat3 &H,
			const Vec2 &x1,
			const Vec2 &x2);

	}//namespace multiview
}  // namespace mvg

#endif  // MVG_MULTIVIEW_HOMOGRAPHY_H_
