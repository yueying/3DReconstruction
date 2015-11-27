#ifndef MVG_MULTIVIEW_SOLVER_ESSENTIAL_KERNEL_H_
#define MVG_MULTIVIEW_SOLVER_ESSENTIAL_KERNEL_H_

#include <vector>
#include "mvg/multiview/two_view_kernel.h"
#include "mvg/multiview/essential.h"
#include "mvg/multiview/solver_fundamental_kernel.h"

using namespace mvg::math;

namespace mvg {
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

			/** 
			* \brief 给出一个通用的5点法对本质矩阵进行估计，通过继承two_view::Kernel
			* \tparam	SolverArg	解决问题模型
			* \tparam	ErrorArg 	度量模型
			* \tparam	ModelArg 	解决问题对应的矩阵类型
			*/
			template<typename SolverArg,typename ErrorArg,typename ModelArg = Mat3>
			class EssentialKernel : public two_view::Kernel < SolverArg, ErrorArg, ModelArg >
			{
			public:
				/**传入左右两幅图像的对应点及对应相机内参*/
				EssentialKernel(const Mat &x1, const Mat &x2, const Mat3 &K1, const Mat3 &K2) :
					two_view::Kernel<SolverArg, ErrorArg, ModelArg>(x1, x2),K1_(K1), K2_(K2) {}
				/**从样本中适配模型*/
				void Fit(const std::vector<size_t> &samples, std::vector<ModelArg> *models) const {
					Mat x1 = extractColumns(this->x1_, samples);
					Mat x2 = extractColumns(this->x2_, samples);

					assert(2 == x1.rows());
					assert(SolverArg::MINIMUM_SAMPLES <= x1.cols());
					assert(x1.rows() == x2.rows());
					assert(x1.cols() == x2.cols());

					// Normalize the data (image coords to camera coords).
					Mat3 K1_inverse = K1_.inverse();
					Mat3 K2_inverse = K2_.inverse();
					Mat x1_normalized, x2_normalized;
					applyTransformationToPoints(x1, K1_inverse, &x1_normalized);
					applyTransformationToPoints(x2, K2_inverse, &x2_normalized);
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
			typedef essential::EssentialKernel <FivePointSolver,
				fundamental::SampsonError, Mat3> FivePointKernel;


		}  // namespace essential
	}  // namespace multiview
}  // namespace mvg

#endif  // MVG_MULTIVIEW_SOLVER_ESSENTIAL_KERNEL_H_
