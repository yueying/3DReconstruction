#ifndef MVG_SFM_LINEAR_PROGRAMMING_INTERFACE_OSI_H_
#define MVG_SFM_LINEAR_PROGRAMMING_INTERFACE_OSI_H_
#include <vector>

#include "OsiClpSolverInterface.hpp"
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"

#include "mvg/math/numeric.h"
#include "mvg/sfm/linear_programming_interface.h"

/* Constraint type codes  (internal) */
#define ROWTYPE_EMPTY            0
#define ROWTYPE_LE               1
#define ROWTYPE_GE               2
#define ROWTYPE_EQ               3
#define ROWTYPE_CONSTRAINT       ROWTYPE_EQ  /* This is the mask for modes */
#define ROWTYPE_OF               4
#define ROWTYPE_INACTIVE         8
#define ROWTYPE_RELAX           16
#define ROWTYPE_GUB             32
#define ROWTYPE_OFMAX            (ROWTYPE_OF + ROWTYPE_GE)
#define ROWTYPE_OFMIN            (ROWTYPE_OF + ROWTYPE_LE)
#define ROWTYPE_CHSIGN           ROWTYPE_GE

/* Public constraint codes */
#define FR                       ROWTYPE_EMPTY
#define LE                       ROWTYPE_LE
#define GE                       ROWTYPE_GE
#define EQ                       ROWTYPE_EQ
#define OF                       ROWTYPE_OF

namespace mvg   {
	namespace sfm  {

		/**	线性求解器基于osi的包装
		 */
		template<typename SolverInterfaceT>
		class OSI_X_SolverWrapper : public LP_Solver
		{
		public:
			OSI_X_SolverWrapper(int parameter_num);

			~OSI_X_SolverWrapper();

			/// 继承函数
			/**
			* \brief	设定线性规划问题的相关约束
			*
			* \param	constraints	指定存放线性规划问题的相关约束
			*
			* \return	true if it succeeds, false if it fails.
			*/
			bool setup(const LP_Constraints & constraints);
			bool setup(const LP_Constraints_Sparse & constraints);

			/**
			* \brief	计算最适合约束条件的可行解
			*
			* \return	true if it succeeds, false if it fails.
			*/
			bool solve();

			/**
			* \brief	求解线性规划问题之后获得相应解
			*
			* \param [in,out]	estimated_params	计算得到的相应解
			*
			* \return	true if it succeeds, false if it fails.
			*/
			bool getSolution(std::vector<double> & estimated_params);

		private:
			SolverInterfaceT *si;
		};

		// 采用osi_clp求解问题
		typedef OSI_X_SolverWrapper<OsiClpSolverInterface> OSI_CLP_SolverWrapper;

		template<typename SolverInterfaceT>
		OSI_X_SolverWrapper<SolverInterfaceT>::OSI_X_SolverWrapper(int parameter_num) : LP_Solver(parameter_num)
		{
			si = new SolverInterfaceT;
			si->setLogLevel(0);
		}

		template<typename SolverInterfaceT>
		OSI_X_SolverWrapper<SolverInterfaceT>::~OSI_X_SolverWrapper()
		{
			// 资源释放
			if (si != NULL)
			{
				delete si;
				si = NULL;
			}
		}

		template<typename SolverInterfaceT>
		bool OSI_X_SolverWrapper<SolverInterfaceT>::setup(const LP_Constraints & cstraints) //cstraints <-> constraints
		{
			bool is_ok = true;
			if (si == NULL)
			{
				return false;
			}
			assert(parameter_num_ == cstraints.parameter_num_);


			const unsigned int kNumVar = cstraints.constraint_mat_.cols();

			std::vector<double> col_lb(kNumVar);//列的下届
			std::vector<double> col_ub(kNumVar);//列的上届

			this->parameter_num_ = kNumVar;

			if (cstraints.is_minimize_)
			{
				si->setObjSense(1);
			}
			else
			{
				si->setObjSense(-1);
			}

			const Mat &A = cstraints.constraint_mat_;

			//Equality constraint will be handked by two constraintsdue to the API limitation.
			size_t nbLine = A.rows() + std::count(cstraints.vec_constrained_type_.begin(), cstraints.vec_constrained_type_.end(), EQ);

			std::vector<double> row_lb(nbLine);//the row lower bounds
			std::vector<double> row_ub(nbLine);//the row upper bounds

			CoinPackedMatrix * matrix = new CoinPackedMatrix(false, 0, 0);
			matrix->setDimensions(0, kNumVar);


			//-- Add row-wise constraint
			size_t indexRow = 0;
			for (int i = 0; i < A.rows(); ++i)
			{
				Vec temp = A.row(i);

				CoinPackedVector row;
				if (cstraints.vec_constrained_type_[i] == EQ || cstraints.vec_constrained_type_[i] == LE)
				{
					int coef = 1;
					for (int j = 0; j < A.cols(); j++)
					{
						row.insert(j, coef * temp.data()[j]);
					}
					row_lb[indexRow] = -1.0 * si->getInfinity();
					row_ub[indexRow] = coef * cstraints.constraint_num_(i);
					matrix->appendRow(row);
					indexRow++;
				}
				if (cstraints.vec_constrained_type_[i] == EQ || cstraints.vec_constrained_type_[i] == GE)
				{
					int coef = -1;
					for (int j = 0; j < A.cols(); j++)
					{
						row.insert(j, coef * temp.data()[j]);
					}
					row_lb[indexRow] = -1.0 * si->getInfinity();
					row_ub[indexRow] = coef * cstraints.constraint_num_(i);
					matrix->appendRow(row);
					indexRow++;
				}
			}

			//-- Setup bounds
			if (cstraints.vec_bounds_.size() == 1)
			{
				// Setup the same bound for all the parameter
				for (int i = 0; i < this->parameter_num_; ++i)
				{
					col_lb[i] = cstraints.vec_bounds_[0].first;
					col_ub[i] = cstraints.vec_bounds_[0].second;
				}
			}
			else
			{

				for (int i = 0; i < this->parameter_num_; ++i)
				{
					col_lb[i] = cstraints.vec_bounds_[i].first;
					col_ub[i] = cstraints.vec_bounds_[i].second;
				}
			}

			si->loadProblem(*matrix, &col_lb[0], &col_ub[0], cstraints.objective_function_coeff_.empty() ? NULL : &cstraints.objective_function_coeff_[0], &row_lb[0], &row_ub[0]);

			delete matrix;

			return is_ok;
		}

		template<typename SolverInterfaceT>
		bool OSI_X_SolverWrapper<SolverInterfaceT>::setup(const LP_Constraints_Sparse & cstraints) //cstraints <-> constraints
		{
			bool is_ok = true;
			if (si == NULL)
			{
				return false;
			}
			assert(parameter_num_ == cstraints.parameter_num_);


			int kNumVar = cstraints.constraint_mat_.cols();
			std::vector<double> col_lb(kNumVar);//the column lower bounds
			std::vector<double> col_ub(kNumVar);//the column upper bounds

			this->parameter_num_ = kNumVar;

			if (cstraints.is_minimize_)
			{
				si->setObjSense(1);
			}
			else
			{
				si->setObjSense(-1);
			}

			const RSparseMat & A = cstraints.constraint_mat_;

			//Equality constraint will be handked by two constraintsdue to the API limitation.
			size_t nbLine = A.rows() + std::count(cstraints.vec_constrained_type_.begin(), cstraints.vec_constrained_type_.end(), EQ);

			std::vector<double> row_lb(nbLine);//the row lower bounds
			std::vector<double> row_ub(nbLine);//the row upper bounds

			CoinPackedMatrix * matrix = new CoinPackedMatrix(false, 0, 0);
			matrix->setDimensions(0, kNumVar);

			//-- Add row-wise constraint
			size_t rowindex = 0;
			for (int i = 0; i < A.rows(); ++i)
			{
				std::vector<int> vec_colno;
				std::vector<double> vec_value;
				for (RSparseMat::InnerIterator it(A, i); it; ++it)
				{
					vec_colno.push_back(it.col());
					vec_value.push_back(it.value());
				}


				if (cstraints.vec_constrained_type_[i] == EQ || cstraints.vec_constrained_type_[i] == LE)
				{
					int coef = 1;
					row_lb[rowindex] = -1.0 * si->getInfinity();
					row_ub[rowindex] = coef * cstraints.constraint_num_(i);
					matrix->appendRow(vec_colno.size(),
						&vec_colno[0],
						&vec_value[0]);
					rowindex++;
				}

				if (cstraints.vec_constrained_type_[i] == EQ || cstraints.vec_constrained_type_[i] == GE)
				{
					int coef = -1;
					for (std::vector<double>::iterator iter_val = vec_value.begin();
						iter_val != vec_value.end();
						iter_val++)
					{
						*iter_val *= coef;
					}
					row_lb[rowindex] = -1.0 * si->getInfinity();
					row_ub[rowindex] = coef * cstraints.constraint_num_(i);
					matrix->appendRow(vec_colno.size(),
						&vec_colno[0],
						&vec_value[0]);
					rowindex++;
				}
			}

			//-- Setup bounds
			if (cstraints.vec_bounds_.size() == 1)
			{
				// Setup the same bound for all the parameter
				for (int i = 0; i < this->parameter_num_; ++i)
				{
					col_lb[i] = cstraints.vec_bounds_[0].first;
					col_ub[i] = cstraints.vec_bounds_[0].second;
				}
			}
			else  {
				// Set the required bound per constraint
				for (int i = 0; i < this->parameter_num_; ++i)
				{
					col_lb[i] = cstraints.vec_bounds_[i].first;
					col_ub[i] = cstraints.vec_bounds_[i].second;
				}
			}

			si->loadProblem(
				*matrix,
				&col_lb[0],
				&col_ub[0],
				cstraints.objective_function_coeff_.empty() ? NULL : &cstraints.objective_function_coeff_[0],
				&row_lb[0],
				&row_ub[0]);

			delete matrix;

			return is_ok;
		}

		template<typename SolverInterfaceT>
		bool OSI_X_SolverWrapper<SolverInterfaceT>::solve()
		{
			//-- Compute solution
			if (si != NULL)
			{
				si->initialSolve();
				return si->isProvenOptimal();
			}
			return false;
		}

		template<typename SolverInterfaceT>
		bool OSI_X_SolverWrapper<SolverInterfaceT>::getSolution(std::vector<double> & estimated_params)
		{
			if (si != NULL)
			{
				int n = si->getNumCols();
				const double *solution;
				solution = si->getColSolution();
				for (int i = 0; i < n; i++)
				{
					estimated_params[i] = solution[i];
				}
				return true;
			}
			return false;
		}

	} // namespace sfm
} // namespace mvg


#endif // MVG_SFM_LINEAR_PROGRAMMING_INTERFACE_OSI_H_

