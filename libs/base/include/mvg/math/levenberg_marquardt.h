//
// [1] K. Madsen, H. Nielsen, O. Tingleoff. Methods for Non-linear Least
// Squares Problems.
// http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf
//
// TODO(keir): Cite the Lourakis' dogleg paper.

#ifndef MVG_NUMERIC_LEVENBERG_MARQUARDT_H
#define MVG_NUMERIC_LEVENBERG_MARQUARDT_H

#include <cmath>

#include "mvg/math/numeric.h"
#include "mvg/math/function_derivative.h"
#include "mvg/utils/notify.h"

namespace mvg {
	namespace math{
		using namespace mvg::utils;
		template<typename Function,
			typename Jacobian = NumericJacobian<Function>,
			typename Solver = Eigen::PartialPivLU <
			Matrix<typename Function::FMatrixType::RealScalar,
			Function::XMatrixType::RowsAtCompileTime,
			Function::XMatrixType::RowsAtCompileTime> > >
		class LevenbergMarquardt {
		public:
			typedef typename Function::XMatrixType::RealScalar Scalar;
			typedef typename Function::FMatrixType FVec;
			typedef typename Function::XMatrixType Parameters;
			typedef Matrix < typename Function::FMatrixType::RealScalar,
				Function::FMatrixType::RowsAtCompileTime,
				Function::XMatrixType::RowsAtCompileTime > JMatrixType;
			typedef Matrix < typename JMatrixType::RealScalar,
				JMatrixType::ColsAtCompileTime,
				JMatrixType::ColsAtCompileTime > AMatrixType;

			// TODO(keir): Some of these knobs can be derived from each other and
			// removed, instead of requiring the user to set them.
			enum Status {
				RUNNING,
				GRADIENT_TOO_SMALL,            // eps > max(J'*f(x))
				RELATIVE_STEP_SIZE_TOO_SMALL,  // eps > ||dx|| / ||x||
				WRONG_TOO_SMALL,               // eps > ||f(x)||
				HIT_MAX_ITERATIONS,
			};

			LevenbergMarquardt(const Function &f)
				: f_(f), df_(f) {}

			struct SolverParameters {
				SolverParameters()
					: gradient_threshold(1e-16),
					relative_step_threshold(1e-16),
					error_threshold(1e-16),
					initial_scale_factor(1e-3),
					max_iterations(100) {}
				Scalar gradient_threshold;       // eps > max(J'*f(x))
				Scalar relative_step_threshold;  // eps > ||dx|| / ||x||
				Scalar error_threshold;          // eps > ||f(x)||
				Scalar initial_scale_factor;     // Initial u for solving normal equations.
				int    max_iterations;           // Maximum number of solver iterations.
			};

			struct Results {
				Scalar error_magnitude;     // ||f(x)||
				Scalar gradient_magnitude;  // ||J'f(x)||
				int    iterations;
				Status status;
			};

			Status Update(const Parameters &x, const SolverParameters &params,
				JMatrixType *J, AMatrixType *A, FVec *error, Parameters *g) {
				*J = df_(x);
				*A = (*J).transpose() * (*J);
				*error = -f_(x);
				*g = (*J).transpose() * *error;
				if (g->array().abs().maxCoeff() < params.gradient_threshold) {
					return GRADIENT_TOO_SMALL;
				}
				else if (error->norm() < params.error_threshold) {
					return WRONG_TOO_SMALL;
				}
				return RUNNING;
			}

			Results minimize(Parameters *x_and_min) {
				SolverParameters params;
				minimize(params, x_and_min);
			}

			Results minimize(const SolverParameters &params, Parameters *x_and_min) {
				Parameters &x = *x_and_min;
				JMatrixType J;
				AMatrixType A;
				FVec error;
				Parameters g;

				Results results;
				results.status = Update(x, params, &J, &A, &error, &g);

				Scalar u = Scalar(params.initial_scale_factor*A.diagonal().maxCoeff());
				Scalar v = 2;

				Parameters dx, x_new;
				int i;
				for (i = 0; results.status == RUNNING && i < params.max_iterations; ++i) {
					notify(INFO) << "iteration: " << i;
					notify(INFO) << "||f(x)||: " << f_(x).norm();
					notify(INFO) << "max(g): " << g.array().abs().maxCoeff();
					notify(INFO) << "u: " << u;
					notify(INFO) << "v: " << v;

					AMatrixType A_augmented = A + u*AMatrixType::Identity(J.cols(), J.cols());
					Solver solver(A_augmented);
					dx = solver.solve(g);
					bool solved = (A_augmented * dx).isApprox(g);
					if (!solved) notify(WRONG) << "Failed to solve";
					if (solved && dx.norm() <= params.relative_step_threshold * x.norm()) {
						results.status = RELATIVE_STEP_SIZE_TOO_SMALL;
						break;
					}
					if (solved) {
						x_new = x + dx;
						// Rho is the ratio of the actual reduction in error to the reduction
						// in error that would be obtained if the problem was linear.
						// See [1] for details.
						Scalar rho((error.squaredNorm() - f_(x_new).squaredNorm())
							/ dx.dot(u*dx + g));
						if (rho > 0) {
							// Accept the Gauss-Newton step because the linear model fits well.
							x = x_new;
							results.status = Update(x, params, &J, &A, &error, &g);
							Scalar tmp = Scalar(2 * rho - 1);
							u = u*std::max(1 / 3., 1 - (tmp*tmp*tmp));
							v = 2;
							continue;
						}
					}
					// Reject the update because either the normal equations failed to solve
					// or the local linear model was not good (rho < 0). Instead, increase u
					// to move closer to gradient descent.
					u *= v;
					v *= 2;
				}
				if (results.status == RUNNING) {
					results.status = HIT_MAX_ITERATIONS;
				}
				results.error_magnitude = error.norm();
				results.gradient_magnitude = g.norm();
				results.iterations = i;
				return results;
			}

		private:
			const Function &f_;
			Jacobian df_;
		};
	}
}  // namespace mv

#endif  // MVG_NUMERIC_LEVENBERG_MARQUARDT_H
