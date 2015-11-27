#ifndef MVG_SFM_BISECTIONLP_LINEAR_PROGRAMMING_H_
#define MVG_SFM_BISECTIONLP_LINEAR_PROGRAMMING_H_

#include <iostream>
#include <iterator>
#include <vector>
#include "mvg/sfm/linear_programming_interface.h"
#include "mvg/utils/notify.h"

namespace mvg   {
	namespace sfm  {

		/**	二分法求解线性方程，迭代到达到精度或者迭代一定的次数
		 */
		template <typename ConstraintBuilder, typename ConstraintType>
		bool BisectionLinearProgramming(
			LP_Solver &solver,
			ConstraintBuilder &constraint_builder,
			std::vector<double> *parameters,
			double gamma_up = 1.0,  // Upper bound
			double gamma_low = 0.0,  // lower bound
			double eps = 1e-8, // 达到的精度，则停止迭代
			const int max_iteration = 20, // 最大的迭代次数
			double *best_feasible_gamma = NULL, // value of best bisection found value
			bool is_verbose = false)
		{
			int k = 0;
			bool is_model_found = false;
			ConstraintType constraint;
			do
			{
				++k; // One more iteration

				double gamma = (gamma_low + gamma_up) / 2.0;

				//-- Setup constraint and solver
				constraint_builder.Build(gamma, constraint);
				solver.setup(constraint);
				//--
				// Solving
				bool bFeasible = solver.solve();
				//--

				if (bFeasible)
				{
					gamma_up = gamma;
					if (best_feasible_gamma)
						*best_feasible_gamma = gamma;
					solver.getSolution(*parameters);
					is_model_found = true;

					if (is_verbose)
						MVG_INFO << "\n" << k << "/" << max_iteration
						<< "\t gamma " << gamma
						<< "\t gamma_up-gamma_low " << gamma_up - gamma_low << std::endl;
				}
				else
				{
					gamma_low = gamma;
					if (is_verbose)
						MVG_INFO << "\nNot feasible with gamma: " << gamma << std::endl;
				}
			} while (k < max_iteration && gamma_up - gamma_low > eps);

			return is_model_found;
		}

	} // namespace sfm
} // namespace mvg


#endif // MVG_SFM_BISECTIONLP_LINEAR_PROGRAMMING_H_
