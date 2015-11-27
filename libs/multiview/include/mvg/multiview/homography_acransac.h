#ifndef MVG_MULTIVIEW_HOMOGRAPHY_ACRANSAC_H_
#define MVG_MULTIVIEW_HOMOGRAPHY_ACRANSAC_H_

#include <limits>

#include "mvg/multiview/solver_homography_kernel.h"
#include "mvg/feature/estimator_acransac.h"
#include "mvg/feature/estimator_acransac_kernel_adaptator.h"

using namespace mvg::feature;

namespace mvg {
	namespace multiview{

		//-- A contrario Functor to filter putative corresponding points
		//--  thanks estimation of the homography matrix.
		struct GeometricFilter_HMatrix_AC
		{
			GeometricFilter_HMatrix_AC(
				double dPrecision = std::numeric_limits<double>::infinity(),
				size_t iteration = 4096)
				: m_dPrecision(dPrecision), m_stIteration(iteration)  {};

			/// Robust fitting of the HOMOGRAPHY matrix
			void Fit(
				const Mat & xA,
				const std::pair<size_t, size_t> & imgSizeA,
				const Mat & xB,
				const std::pair<size_t, size_t> & imgSizeB,
				std::vector<size_t> & vec_inliers) const
			{
				vec_inliers.clear();

				// Define the AContrario adapted Homography matrix solver
				typedef ACKernelAdaptor <
					mvg::multiview::homography::FourPointSolver,
					mvg::multiview::homography::AsymmetricError,
					UnnormalizerI,
					Mat3 >
					KernelType;

				KernelType kernel(
					xA, imgSizeA.first, imgSizeA.second,
					xB, imgSizeB.first, imgSizeB.second,
					false); // configure as point to point error model.

				// Robustly estimate the Homography matrix with A Contrario ransac
				Mat3 H;
				double upper_bound_precision = m_dPrecision;
				std::pair<double, double> acransac_out =
					ACRANSAC(kernel, vec_inliers, m_stIteration, &H, upper_bound_precision);

				if (vec_inliers.size() < KernelType::MINIMUM_SAMPLES *2.5)  {
					vec_inliers.clear();
				}
			}

			double m_dPrecision;  //upper_bound of the precision
			size_t m_stIteration; //maximal number of used iterations
		};
	}//namespace multiview
} // namespace mvg

#endif // MVG_MULTIVIEW_HOMOGRAPHY_ACRANSAC_H_
