#include <algorithm>
#include <iostream>
#include <vector>

#include "testing.h"
#include "mvg/sfm/linear_programming_osi.h"
using namespace mvg::sfm;


/**	 线性规划问题
 *  max(143x + 60y)
 *  约束：
 *  120x + 210y <= 15000
 *  110x + 30y <= 4000
 *  x + y <= 75
 *  x >= 0
 *  y >= 0
 */
void BuildLinearProblem(LP_Constraints &cstraint)
{
  cstraint.parameter_num_ = 2;
  cstraint.is_minimize_ = false;

  //配置目标函数
  cstraint.objective_function_coeff_.push_back(143);
  cstraint.objective_function_coeff_.push_back(60);

  cstraint.constraint_mat_ = Mat(5,2);
  cstraint.constraint_mat_ <<
    120, 210,
    110, 30,
    1, 1,
    1, 0,
    0, 1;

  cstraint.constraint_num_ = Vec(5);
  cstraint.constraint_num_ << 15000, 4000, 75, 0, 0;

  cstraint.vec_constrained_type_.resize(5);
  std::fill_n(cstraint.vec_constrained_type_.begin(), 3, LP_Constraints::LP_LESS_OR_EQUAL);
  std::fill_n(cstraint.vec_constrained_type_.begin()+3, 2, LP_Constraints::LP_GREATER_OR_EQUAL);

  cstraint.vec_bounds_ = std::vector< std::pair<double,double> >(5);
  std::fill(cstraint.vec_bounds_.begin(),cstraint.vec_bounds_.end(),
      std::make_pair((double)-1e+30, (double)1e+30));
}

TEST(LinearProgramming, OsiclpDenseSample) {

  LP_Constraints cstraint;
  BuildLinearProblem(cstraint);

  //求解
  std::vector<double> vec_solution(2);
  OSI_CLP_SolverWrapper solver(2);
  solver.setup(cstraint);

  EXPECT_TRUE(solver.solve());
  solver.getSolution(vec_solution);

  EXPECT_NEAR( 21.875000, vec_solution[0], 1e-6);
  EXPECT_NEAR( 53.125000, vec_solution[1], 1e-6);
}

/**	maximize :
 * 3 x0 + 1 x1 + 5 x2 + 1 x3
 * 约束
 * 3 x0 + 1 x1 + 2 x2         = 30
 * 2 x0 + 1 x1 + 3 x2 + 1 x3 >= 15
 *        2 w1        + 3 x3 <= 25
 * 边界
 * 0 <= x0, x2, x3 < infinity
 * 0 <= x1 <= 10
 */
void BuildSparseLinearProblem(LP_Constraints_Sparse & cstraint)
{
  // 待求解参数个数
  cstraint.parameter_num_ = 4; // {x0, x1, x2, x3}

  // 将约束条件构件稀释矩阵
  RSparseMat & A = cstraint.constraint_mat_;
  A.resize(3,4);
  A.coeffRef(0,0) = 3;
  A.coeffRef(0,1) = 1;
  A.coeffRef(0,2) = 2;

  A.coeffRef(1,0) = 2;
  A.coeffRef(1,1) = 1;
  A.coeffRef(1,2) = 3;
  A.coeffRef(1,3) = 1;

  A.coeffRef(2,1) = 2;
  A.coeffRef(2,3) = 3;

  // 约束方程的右值
  Vec & C = cstraint.constraint_num_;
  C.resize(3, 1);
  C[0] = 30;
  C[1] = 15;
  C[2] = 25;

  // 给出约束方程的关系符号
  std::vector<LP_Constraints::LP_Sign> & vec_sign = cstraint.vec_constrained_type_;
  vec_sign.resize(3);
  vec_sign[0] = LP_Constraints::LP_EQUAL;
  vec_sign[1] = LP_Constraints::LP_GREATER_OR_EQUAL;
  vec_sign[2] = LP_Constraints::LP_LESS_OR_EQUAL;

  // 给出变量边界
  cstraint.vec_bounds_ = std::vector< std::pair<double,double> >(4);
  std::fill(cstraint.vec_bounds_.begin(),cstraint.vec_bounds_.end(),
      std::make_pair(0.0, (double)1e+30));
  cstraint.vec_bounds_[1].second = 10;

  // 设定目标函数
  cstraint.is_minimize_ = false;
  cstraint.objective_function_coeff_.resize(4);
  cstraint.objective_function_coeff_[0] = 3;
  cstraint.objective_function_coeff_[1] = 1;
  cstraint.objective_function_coeff_[2] = 5;
  cstraint.objective_function_coeff_[3] = 1;
}

TEST(LinearProgramming, OsiclpSparseSample) {

  LP_Constraints_Sparse cstraint;
  BuildSparseLinearProblem(cstraint);

  //计算求解
  std::vector<double> vec_solution(4);
  OSI_CLP_SolverWrapper solver(4);
  solver.setup(cstraint);

  EXPECT_TRUE(solver.solve());
  solver.getSolution(vec_solution);

  EXPECT_NEAR( 0.00, vec_solution[0], 1e-2);
  EXPECT_NEAR( 0.00, vec_solution[1], 1e-2);
  EXPECT_NEAR( 15, vec_solution[2], 1e-2);
  EXPECT_NEAR( 8.33, vec_solution[3], 1e-2);
}

