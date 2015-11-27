#ifndef MVG_NUMERIC_DERIVATIVE_H
#define MVG_NUMERIC_DERIVATIVE_H

#include <cmath>

#include "mvg/math/numeric.h"
#include "mvg/utils/notify.h"

namespace mvg {
	namespace math{
		// Numeric derivative of a function.
		// TODO(keir): Consider adding a quadratic approximation.

		enum NumericJacobianMode {
			CENTRAL,
			FORWARD,
		};

		template<typename Function, NumericJacobianMode mode = CENTRAL>
		class NumericJacobian {
		public:
			typedef typename Function::XMatrixType Parameters;
			typedef typename Function::XMatrixType::RealScalar XScalar;
			typedef typename Function::FMatrixType FMatrixType;
			typedef Matrix < typename Function::FMatrixType::RealScalar,
				Function::FMatrixType::RowsAtCompileTime,
				Function::XMatrixType::RowsAtCompileTime >
				JMatrixType;

			NumericJacobian(const Function &f) : f_(f) {}

			// TODO(keir): Perhaps passing the jacobian back by value is not a good idea.
			JMatrixType operator()(const Parameters &x) {
				// Empirically determined constant.
				Parameters eps = x.array().abs() * XScalar(1e-5);
				// To handle cases where a paremeter is exactly zero, instead use the mean
				// eps for the other dimensions.
				XScalar mean_eps = eps.sum() / eps.rows();
				if (mean_eps == XScalar(0)) {
					// TODO(keir): Do something better here.
					mean_eps = 1e-8; // ~sqrt(machine precision).
				}
				// TODO(keir): Elimininate this needless function evaluation for the
				// central difference case.
				FMatrixType fx = f_(x);
				const int rows = fx.rows();
				const int cols = x.rows();
				JMatrixType jacobian(rows, cols);
				Parameters x_plus_delta = x;
				for (int c = 0; c < cols; ++c) {
					if (eps(c) == XScalar(0)) {
						eps(c) = mean_eps;
					}
					x_plus_delta(c) = x(c) + eps(c);
					jacobian.col(c) = f_(x_plus_delta);

					XScalar one_over_h = 1 / eps(c);
					if (mode == CENTRAL) {
						x_plus_delta(c) = x(c) - eps(c);
						jacobian.col(c) -= f_(x_plus_delta);
						one_over_h /= 2;
					}
					else {
						jacobian.col(c) -= fx;
					}
					x_plus_delta(c) = x(c);
					jacobian.col(c) = jacobian.col(c) * one_over_h;
				}
				return jacobian;
			}
		private:
			const Function &f_;
		};

		template<typename Function, typename Jacobian>
		bool CheckJacobian(const Function &f, const typename Function::XMatrixType &x) {
			Jacobian j_analytic(f);
			NumericJacobian<Function> j_numeric(f);

			typename NumericJacobian<Function>::JMatrixType J_numeric = j_numeric(x);
			typename NumericJacobian<Function>::JMatrixType J_analytic = j_analytic(x);
			notify(INFO) << J_numeric - J_analytic;
			return true;
		}
	}
}  // namespace mvg

#endif  // MVG_NUMERIC_DERIVATIVE_H
