#ifndef MVG_FEATURE_ESTIMATION_LINE_KERNEL_H_
#define MVG_FEATURE_ESTIMATION_LINE_KERNEL_H_

#include "mvg/math/numeric.h"
using namespace mvg::math;

namespace mvg {
	namespace feature{

		struct LineSolver {
			enum { MINIMUM_SAMPLES = 2 };
			enum { MAX_MODELS = 1 };

			/**
			 * \brief	b+ax = y, 通过转换最小二乘利用SVD分解求解(b,a)
			 *
			 * \param	x			 	样本集
			 * \param [in,out]	lines	模型参数，为(b,a)
			 */
			static void Solve(const Mat &x, std::vector<Vec2> *lines)
			{
				Mat X(x.cols(), 2);
				X.col(0).setOnes();
				X.col(1) = x.row(0).transpose();
				Mat A(X.transpose() * X);
				Vec b(X.transpose() * x.row(1).transpose());
				Vec2 ba;
				Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
				ba = svd.solve(b);
				lines->push_back(ba);
			}
		};


		/** \brief	将点带入直线，确定误差 */
		struct PointToLineError {

			/**
			 * \brief	通过方程系数，确定方程的可能解与真实解之间的误差（差值的平方）
			 *
			 * \param	line_coeff		 	方程系数
			 * \param	equation_solution	方程的可能解
			 *
			 * \return	真实解与可能解之间的差值的平方
			 */
			static double Error(const Vec2 &line_coeff, const Vec2 &equation_solution) {
				double b = line_coeff[0];
				double a = line_coeff[1];
				double x = equation_solution[0];
				double y = equation_solution[1];
				double e = y - (a*x + b);
				return e*e;
			}
		};

		
		/** \brief	简单的线性内核，一组观测数据中找出合适的2维直线 */
		struct LineKernel {
			typedef Vec2 Model;  // 直线的参数: a, b;
			enum { MINIMUM_SAMPLES = 2 };

			LineKernel(const Mat2X &xs) : xs_(xs) {}

			/// 样本数目
			size_t NumSamples() const { return static_cast<size_t> (xs_.cols()); }

			/**
			 * \brief	根据给定样本集求解模型系数.
			 *
			 * \param	samples		 	样本集
			 * \param [in,out]	lines	模型（直线）系数
			 */
			void Fit(const std::vector<size_t> &samples, std::vector<Vec2> *lines) const {
				// 确保样本数大小计算模型所需要的最小样本数
				assert(samples.size() >= (unsigned int)MINIMUM_SAMPLES);
				// 采样标准的最小二乘来解决
				Mat2X sampled_xs = extractColumns(xs_, samples);

				LineSolver::Solve(sampled_xs, lines);
			}
						
			/// 根据给定的模型，计算样本点的偏差（真实值与估计值得差值的平方）
			double Error(size_t sample, const Vec2 &ba) const {
				return PointToLineError::Error(ba, xs_.col(sample));
			}

			const Mat2X &xs_;
		};

	} // namespace feature
} // namespace mvg

#endif // MVG_FEATURE_ESTIMATION_LINE_KERNEL_H_
