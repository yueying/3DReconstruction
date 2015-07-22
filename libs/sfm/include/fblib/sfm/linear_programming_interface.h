#ifndef FBLIB_SFM_LINEAR_PROGRAMMING_INTERFACE_H_
#define FBLIB_SFM_LINEAR_PROGRAMMING_INTERFACE_H_

#include <vector>
#include <utility>
#include "fblib/math/numeric.h"
using namespace fblib::math;

namespace fblib {
	namespace sfm  {
		
		
		/** \brief	线性规划问题的通用容器    
		*           主要包括目标函数，约束类型，约束右值，目标函数最大化还是最小化
		*/
		struct LP_Constraints
		{
			enum LP_Sign
			{
				LP_LESS_OR_EQUAL = 1,  // (<=)
				LP_GREATER_OR_EQUAL = 2,  // (>=)
				LP_EQUAL = 3,   // (=)
				LP_FREE = 4 //不受约束
			};

			LP_Constraints() {
				is_minimize_ = false;
			}

			int parameter_num_; //!< 在约束中参数或变量的个数
			Mat constraint_mat_; //!< 约束的矩阵形式表示
			Vec constraint_num_; //!< 约束方程的右值（约束值）
			std::vector<LP_Sign> vec_constrained_type_; //!< 约束类型
			std::vector< std::pair<double, double> > vec_bounds_; //!<约束方程中变量的范围

			bool is_minimize_; //!< 是否求解目标函数的最小值
			std::vector<double> objective_function_coeff_; //!< 目标函数的系数
		};

		/** \brief	线性规划问题，采用稀释矩阵存储
		*           主要包括目标函数，约束类型，约束右值，目标函数最大化还是最小化
		*/
		struct LP_Constraints_Sparse
		{
			LP_Constraints_Sparse() {
				is_minimize_ = false;
			}

			int parameter_num_; //!< 在约束中参数或变量的个数
			std::vector< std::pair<double, double> > vec_bounds_; //!<约束方程中变量的范围

			RSparseMat constraint_mat_; //!< 约束方程的矩阵形式采用稀释矩阵表示，行优先存储
			Vec constraint_num_; //!< 约束方程的右值（约束值）
			std::vector<LP_Constraints::LP_Sign> vec_constrained_type_; //!< 约束类型

			bool is_minimize_; //!< 是否求解目标函数的最小值
			std::vector<double> objective_function_coeff_; //!< 目标函数的系数
		};

		
		/** \brief	通用线性规划问题求解器，主要包括设置约束，问题求解，得到解 */
		class LP_Solver
		{
		public:

			LP_Solver(int parameter_num) :parameter_num_(parameter_num){};

			/**
			 * \brief	设定线性规划问题的相关约束
			 *
			 * \param	constraints	指定存放线性规划问题的相关约束
			 *
			 * \return	true if it succeeds, false if it fails.
			 */
			virtual bool setup(const LP_Constraints & constraints) = 0;
			virtual bool setup(const LP_Constraints_Sparse & constraints) = 0;

			/**
			 * \brief	计算最适合约束条件的可行解
			 *
			 * \return	true if it succeeds, false if it fails.
			 */
			virtual bool solve() = 0;

			/**
			 * \brief	求解线性规划问题之后获得相应解
			 *
			 * \param [in,out]	estimated_params	计算得到的相应解
			 *
			 * \return	true if it succeeds, false if it fails.
			 */
			virtual bool getSolution(std::vector<double> &estimated_params) = 0;

		protected:
			int parameter_num_; //!<约束方程中参数的个数
		};

	} // namespace sfm
} // namespace fblib


#endif // FBLIB_SFM_LINEAR_PROGRAMMING_INTERFACE_H_
