#ifndef FBLIB_MULTIVIEW_SOLVER_ESSENTIAL_KERNEL_H_
#define FBLIB_MULTIVIEW_SOLVER_ESSENTIAL_KERNEL_H_

#include <vector>
#include "fblib/multiview/two_view_kernel.h"
#include "fblib/multiview/essential.h"
#include "fblib/multiview/solver_fundamental_kernel.h"

using namespace fblib::math;

namespace fblib {
	namespace multiview {
		namespace essential {

			/**
			 * Eight-point algorithm for solving for the essential matrix from normalized
			 * image coordinates of point correspondences.
			 * See page 294 in HZ Result 11.1.
			 *
			 */
			struct PointWithNegativeDepth {
				enum { MINIMUM_SAMPLES = 8 };
				enum { MAX_MODELS = 1 };
				static void Solve(const Mat &x1, const Mat &x2, std::vector<Mat3> *E);
			};

			/**
			 * 通过对应的5点来进行本质矩阵的计算
			 * 输入的点必须进行归一化处理
			 */
			struct FivePointSolver {
				enum { MINIMUM_SAMPLES = 5 };
				enum { MAX_MODELS = 10 };
				static void Solve(const Mat &x1, const Mat &x2, std::vector<Mat3> *E);
			};

			//-- Generic Solver for the 5pt Essential Matrix Estimation.
			//-- Need a new Class that inherit of two_view::kernel::kernel.
			//    Error must be overwrite in order to compute fundamental_matrix from E and K's.
			//-- Fitting must normalize image values to camera values.
			template<typename SolverArg,
				typename ErrorArg,
				typename ModelArg = Mat3>
			class EssentialKernel :
				public two_view::Kernel < SolverArg, ErrorArg, ModelArg >
			{
			public:
				EssentialKernel(const Mat &x1, const Mat &x2,
					const Mat3 &K1, const Mat3 &K2) :
					two_view::Kernel<SolverArg, ErrorArg, ModelArg>(x1, x2),
					K1_(K1), K2_(K2) {}
				void Fit(const std::vector<size_t> &samples, std::vector<ModelArg> *models) const {
					Mat x1 = ExtractColumns(this->x1_, samples);
					Mat x2 = ExtractColumns(this->x2_, samples);

					assert(2 == x1.rows());
					assert(SolverArg::MINIMUM_SAMPLES <= x1.cols());
					assert(x1.rows() == x2.rows());
					assert(x1.cols() == x2.cols());

					// Normalize the data (image coords to camera coords).
					Mat3 K1Inverse = K1_.inverse();
					Mat3 K2Inverse = K2_.inverse();
					Mat x1_normalized, x2_normalized;
					ApplyTransformationToPoints(x1, K1Inverse, &x1_normalized);
					ApplyTransformationToPoints(x2, K2Inverse, &x2_normalized);
					SolverArg::Solve(x1_normalized, x2_normalized, models);
				}

				double Error(size_t sample, const ModelArg &model) const {
					Mat3 fundamental_matrix;
					FundamentalFromEssential(model, K1_, K2_, &fundamental_matrix);
					return ErrorArg::Error(fundamental_matrix, this->x1_.col(sample), this->x2_.col(sample));
				}
			protected:
				Mat3 K1_, K2_; //两个标定的相机内参
			};

			// 8点法进行本质矩阵估计
			typedef essential::EssentialKernel < PointWithNegativeDepth,
				fundamental::SampsonError, Mat3 > EightPointKernel;


			// 5点法进行本质矩阵估计
			typedef essential::EssentialKernel < FivePointSolver,
				fundamental::SampsonError, Mat3 > FivePointKernel;


		}  // namespace essential
	}  // namespace multiview
}  // namespace fblib

#endif  // FBLIB_MULTIVIEW_SOLVER_ESSENTIAL_KERNEL_H_
