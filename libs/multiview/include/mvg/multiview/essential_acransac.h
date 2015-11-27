#ifndef MVG_MULTIVIEW_ESSENTIAL_ACRANSAC_H_
#define MVG_MULTIVIEW_ESSENTIAL_ACRANSAC_H_

#include <limits>
#include "mvg/multiview/solver_essential_kernel.h"
#include "mvg/multiview/essential.h"
#include "mvg/feature/estimator_acransac.h"
#include "mvg/feature/estimator_acransac_kernel_adaptator.h"

using namespace mvg::feature;

namespace mvg {
	namespace multiview{

		//-- A contrario Functor to filter putative corresponding points
		//--  thanks estimation of the essential matrix.
		//- Suppose that all image have the same K matrix
		struct GeometricFilter_EMatrix_AC
		{
			GeometricFilter_EMatrix_AC(
				const Mat3 & K,
				double dPrecision = std::numeric_limits<double>::infinity(),
				size_t iteration = 4096)
				: m_dPrecision(dPrecision), max_iteration(iteration), camera_matrix(K) {};

			/// Robust fitting of the ESSENTIAL matrix
			void Fit(
				const Mat & xA,
				const std::pair<size_t, size_t> & imgSizeA,
				const Mat & xB,
				const std::pair<size_t, size_t> & imgSizeB,
				std::vector<size_t> & vec_inliers) const
			{
				vec_inliers.clear();

				// Define the AContrario adapted Essential matrix solver
				typedef ACKernelAdaptorEssential <
					mvg::multiview::essential::FivePointKernel,
					mvg::multiview::fundamental::EpipolarDistanceError,
					UnnormalizerT,
					Mat3 >
					KernelType;

				KernelType kernel(xA, imgSizeA.first, imgSizeA.second,
					xB, imgSizeB.first, imgSizeB.second,
					camera_matrix, camera_matrix);

				// Robustly estimate the Essential matrix with A Contrario ransac
				Mat3 E;
				double upper_bound_precision = m_dPrecision;
				std::pair<double, double> acransac_out =
					ACRANSAC(kernel, vec_inliers, max_iteration, &E, upper_bound_precision);

				if (vec_inliers.size() < KernelType::MINIMUM_SAMPLES *2.5)  {
					vec_inliers.clear();
				}
			}

			double m_dPrecision;  //upper_bound of the precision
			size_t max_iteration; //!< 最大的迭代数
			Mat3 camera_matrix; //!< 相机内参矩阵
		};
	}// namespace multiview
} // namespace mvg


#endif // MVG_MULTIVIEW_ESSENTIAL_ACRANSAC_H_
