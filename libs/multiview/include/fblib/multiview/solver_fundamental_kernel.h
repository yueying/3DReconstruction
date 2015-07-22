#ifndef FBLIB_MULTIVIEW_SOLVER_FUNDAMENTAL_KERNEL_H_
#define FBLIB_MULTIVIEW_SOLVER_FUNDAMENTAL_KERNEL_H_

#include <vector>
#include "fblib/math/numeric.h"
#include "fblib/multiview/two_view_kernel.h"

using namespace fblib::math;

namespace fblib {
	namespace multiview {
		namespace fundamental {

			/**
			 * \brief 采用7个点对来估计基础矩阵
			 *		  由方程Af = 0，系数矩阵A一般情况秩为7， 则对于方程的解集是 9 维空间中
			 *		  的通过坐标原点的一张 2 维	平面，令f_1,f_2是方程的两个单位正交解，
			 *		  则它的单位解集合是一个单参数簇f=sf_1+(1-s)f_2	  
			 *        于是基础矩阵可以表示为F=sF_1+(1-s)F_2,由于基础矩阵的秩为2，因此获得
			 *        参数s的一个约束方程det(sF_1+(1-s)F_2)=0.
			 *		  这是一个关于参数s的3次方程，因此有一个解或者有3个解。
			 *        注：若图像中仅有7个对应点，当基础矩阵有3个解时，不能断定是哪一个是真解
			 *           若图像中有多于7个对应点，当基础矩阵有3个解时，可选取匹配点对数最多的矩阵
			 *           作为基础矩阵
			 *
			 * \see http://www.cs.unc.edu/~marc/tutorial/node55.html
			 */
			struct SevenPointSolver {
				enum { MINIMUM_SAMPLES = 7 };
				enum { MAX_MODELS = 3 };
				static void Solve(const Mat &x1, const Mat &x2, std::vector<Mat3> *fundamental_matrix);
			};

			/**
			* \brief 采用8个点对来估计基础矩阵
			*/
			struct EightPointSolver {
				enum { MINIMUM_SAMPLES = 8 };
				enum { MAX_MODELS = 1 };
				static void Solve(const Mat &x1, const Mat &x2, std::vector<Mat3> *Fs);
			};

			/**
			 *  \brief 用来构建基础矩阵的约束方程，给定一对点对应m=(u,v,1)T,m'=(u',v',1)T
			 *  	   满足基础矩阵F m'TFm=0,令F=(f_ij),则约束方程可以化简为：
			 *  	    u'uf_11+u'vf_12+u'f_13+v'uf_21+v'vf_22+v'f_23+uf_31+vf_32+f_33=0
			 *  	    令f = (f_11,f_12,f_13,f_21,f_22,f_23,f_31,f_32,f_33)
			 *  	    则(u'u,u'v,u',v'u,v'v,v',u,v,1)f=0;
			 *  	    这样，给定N个对应点就可以得到线性方程组Af=0
			 *  	    A就是一个N*9的矩阵，由于基础矩阵是非零的，所以f是一个非零向量，即
			 *  	    线性方程组有非零解，另外基础矩阵的秩为2，重要的约束条件
			 */
			template<typename TMatX, typename TMatA>
			inline void EncodeEpipolarEquation(const TMatX &x1, const TMatX &x2, TMatA *A) {
				for (int i = 0; i < x1.cols(); ++i) {
					(*A)(i, 0) = x2(0, i) * x1(0, i);  // 0 代表 x 的坐标,
					(*A)(i, 1) = x2(0, i) * x1(1, i);  // 1 代表 y 的坐标.
					(*A)(i, 2) = x2(0, i);
					(*A)(i, 3) = x2(1, i) * x1(0, i);
					(*A)(i, 4) = x2(1, i) * x1(1, i);
					(*A)(i, 5) = x2(1, i);
					(*A)(i, 6) = x1(0, i);
					(*A)(i, 7) = x1(1, i);
					(*A)(i, 8) = 1.0;
				}
			}

			/// Compute SampsonError related to the Fundamental matrix and 2 correspondences
			struct SampsonError {
				static double Error(const Mat3 &fundamental_matrix, const Vec2 &x1, const Vec2 &x2) {
					Vec3 x(x1(0), x1(1), 1.0);
					Vec3 y(x2(0), x2(1), 1.0);
					// See page 287 equation (11.9) of HZ.
					Vec3 F_x = fundamental_matrix * x;
					Vec3 Ft_y = fundamental_matrix.transpose() * y;
					return Square(y.dot(F_x)) / (F_x.head<2>().squaredNorm()
						+ Ft_y.head<2>().squaredNorm());
				}
			};

			struct SymmetricEpipolarDistanceError {
				static double Error(const Mat3 &fundamental_matrix, const Vec2 &x1, const Vec2 &x2) {
					Vec3 x(x1(0), x1(1), 1.0);
					Vec3 y(x2(0), x2(1), 1.0);
					// See page 288 equation (11.10) of HZ.
					Vec3 F_x = fundamental_matrix * x;
					Vec3 Ft_y = fundamental_matrix.transpose() * y;
					return Square(y.dot(F_x)) * (1.0 / F_x.head<2>().squaredNorm()
						+ 1.0 / Ft_y.head<2>().squaredNorm())
						/ 4.0;  // The divide by 4 is to make this match the Sampson distance.
				}
			};

			struct EpipolarDistanceError {
				static double Error(const Mat3 &fundamental_matrix, const Vec2 &x1, const Vec2 &x2) {
					// Transfer error in image 2
					// See page 287 equation (11.9) of HZ.
					Vec3 x(x1(0), x1(1), 1.0);
					Vec3 y(x2(0), x2(1), 1.0);
					Vec3 F_x = fundamental_matrix * x;
					return Square(F_x.dot(y)) / F_x.head<2>().squaredNorm();
				}
			};
			typedef EpipolarDistanceError SimpleError;

			//-- Kernel solver for the 8pt Fundamental Matrix Estimation
			typedef two_view::Kernel < SevenPointSolver, SampsonError, Mat3 >
				SevenPointKernel;

			//通过8点算法进行基础矩阵估计
			typedef two_view::Kernel<EightPointSolver, SampsonError, Mat3> EightPointKernel;

			//-- Normalized 7pt kernel -> conditioning from HZ (Algo 11.1) pag 282
			typedef two_view::Kernel <
				two_view::NormalizedSolver<SevenPointSolver, UnnormalizerT>,
				SampsonError,
				Mat3 >
				NormalizedSevenPointKernel;

			//-- Normalized 8pt kernel -> conditioning from HZ (Algo 11.1) pag 282
			typedef two_view::Kernel <
				two_view::NormalizedSolver<EightPointSolver, UnnormalizerT>,
				SampsonError,
				Mat3 >
				NormalizedEightPointKernel;

		}  // namespace fundamental
	}  // namespace multiview
}  // namespace fblib

#endif  // FBLIB_MULTIVIEW_SOLVER_FUNDAMENTAL_KERNEL_H_
