#pragma once

#include "fblib/multiview/solver_fundamental_kernel.h"
#include "fblib/multiview/essential.h"
#include "fblib/feature/estimator_acransac.h"
#include "fblib/feature/estimator_acransac_kernel_adaptator.h"
#include <limits>
using namespace fblib::feature;
namespace fblib {
	namespace multiview{

		//-- A contrario Functor to filter putative corresponding points
		struct GeometricFilter_FMatrix_AC
		{
			GeometricFilter_FMatrix_AC(
				double dPrecision = std::numeric_limits<double>::infinity(),
				size_t iteration = 4096)
				: m_dPrecision(dPrecision), max_iteration(iteration) {};

			/// Robust fitting of the FUNDAMENTAL matrix
			void Fit(
				const Mat & xA,
				const std::pair<size_t, size_t> & imgSizeA,
				const Mat & xB,
				const std::pair<size_t, size_t> & imgSizeB,
				std::vector<size_t> & vec_inliers) const
			{
				vec_inliers.clear();
				// Define the AContrario adapted Fundamental matrix solver
				typedef ACKernelAdaptor <
					fblib::multiview::fundamental::SevenPointSolver,
					fblib::multiview::fundamental::SimpleError,
					UnnormalizerT,
					Mat3 >
					KernelType;

				KernelType kernel(xA, imgSizeA.first, imgSizeA.second,
					xB, imgSizeB.first, imgSizeB.second, true);

				// Robustly estimate the Fundamental matrix with A Contrario ransac
				Mat3 fundamental_matrix;
				double upper_bound_precision = m_dPrecision;
				std::pair<double, double> acransac_out =
					ACRANSAC(kernel, vec_inliers, max_iteration, &fundamental_matrix, upper_bound_precision);

				if (vec_inliers.size() < KernelType::MINIMUM_SAMPLES *2.5)  {
					vec_inliers.clear();
				}
			}

			double m_dPrecision;  //upper_bound of the precision
			size_t max_iteration; //maximal number of iteration used
		};
	}
}; // namespace fblib

