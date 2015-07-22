#ifndef FBLIB_MULTIVIEW_TWO_VIEW_KERNEL_H_
#define FBLIB_MULTIVIEW_TWO_VIEW_KERNEL_H_

#include <vector>
#include "fblib/multiview/conditioning.h"
#include "fblib/math/numeric.h"

namespace fblib {
	namespace multiview {
		namespace two_view {

			/**
			 * \brief	给出通用的鲁棒性估计算法，比如根据两幅视图之间的对极几何的关系，对匹配点进行约束
			 * 			
			 * 			1. 模型（Mat3）应该是基础矩阵或者单应矩阵
			 * 			2. 最小的样本点应该是7或8(或者 4).  
			 * 			3. 将样本转换为模型的方法  
			 * 			
			 * 			具体：	
			 * 			  1. Kernel::MAX_MODELS    
			 * 			  2. Kernel::MINIMUM_SAMPLES   
			 * 			  3. Kernel::Fit(vector<size_t>, vector<Kernel::Model> *)
			 * 			  4. Kernel::Error(size_t, Model)
			 * 			
			 *
			 * \tparam	SolverArg	解决问题模型
			 * \tparam	ErrorArg 	度量模型
			 * \tparam	ModelArg 	解决问题对应的矩阵类型
			 */

			template<typename SolverArg,
				typename ErrorArg,
				typename ModelArg = Mat3>
			class Kernel {
			public:
				Kernel(const Mat &x1, const Mat &x2) : x1_(x1), x2_(x2) {}
				typedef SolverArg Solver;
				typedef ModelArg  Model;
				//模型估计需要的最小点数
				enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
				// 返回计算模型数目
				enum { MAX_MODELS = Solver::MAX_MODELS };

				/**	提取所需要的样本，根据样本进行适配模型
				 */
				void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const {
					Mat x1 = ExtractColumns(x1_, samples),
						x2 = ExtractColumns(x2_, samples);
					Solver::Solve(x1, x2, models);
				}
				/**	返回关联的n样本点之间的偏差值
				 */
				double Error(size_t sample, const Model &model) const {
					return ErrorArg::Error(model, x1_.col(sample), x2_.col(sample));
				}
				/**	返回对应点的数目
				 */
				size_t NumSamples() const {
					return x1_.cols();
				}
				/**	根据给定的计算模型
				 */
				static void Solve(const Mat &x1, const Mat &x2, std::vector<Model> *models) {
					// 内核类型可以通过模板进行提供
					Solver::Solve(x1, x2, models);
				}
			protected:
				const Mat & x1_, &x2_; // 左右对应的点
			};

			/**	前面处理的规范化版本
			 */
			template<typename SolverArg,
				typename UnnormalizerArg,
				typename ModelArg = Mat3>
			class NormalizedSolver {
			public:
				enum { MINIMUM_SAMPLES = SolverArg::MINIMUM_SAMPLES };
				enum { MAX_MODELS = SolverArg::MAX_MODELS };

				static void Solve(const Mat &x1, const Mat &x2, std::vector<ModelArg> *models) {
					assert(2 == x1.rows());
					assert(MINIMUM_SAMPLES <= x1.cols());
					assert(x1.rows() == x2.rows());
					assert(x1.cols() == x2.cols());

					// 数据进行规范化处理
					Mat3 T1, T2;
					Mat x1_normalized, x2_normalized;
					NormalizePoints(x1, &x1_normalized, &T1);
					NormalizePoints(x2, &x2_normalized, &T2);

					SolverArg::Solve(x1_normalized, x2_normalized, models);
					//给出模型中不进行归一化的部分
					for (int i = 0; i < models->size(); ++i) {
						UnnormalizerArg::Unnormalize(T1, T2, &(*models)[i]);
					}
				}
			};

		}  // namespace two_view
	}  // namespace multiview
}  // namespace fblib

#endif  // FBLIB_MULTIVIEW_TWO_VIEW_KERNEL_H_
