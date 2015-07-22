#ifndef FBLIB_TRACKING_KALMAN_FILTER_H_
#define FBLIB_TRACKING_KALMAN_FILTER_H_

#include "fblib/math/numeric.h"
using namespace fblib::math;

namespace fblib {
	namespace tracking{
	// A Kalman filter with order N and observation size K.
	template<typename T, int N, int K>
	class KalmanFilter {
	public:
		struct State {
			Eigen::Matrix<T, N, 1> mean;
			Eigen::Matrix<T, N, N> covariance;
		};

		// Initialize from row-major data; convenient for constant initializers.
		KalmanFilter(const T* state_transition_data,
			const T* observation_data,
			const T* process_covariance_data,
			const T* default_measurement_covariance_data)
			: state_transition_matrix_(
			Eigen::Matrix<T, N, N, Eigen::RowMajor>(state_transition_data)),
			observation_matrix_(
			Eigen::Matrix<T, K, N, Eigen::RowMajor>(observation_data)),
			process_covariance_(
			Eigen::Matrix<T, N, N, Eigen::RowMajor>(process_covariance_data)),
			default_measurement_covariance_(
			Eigen::Matrix<T, K, K, Eigen::RowMajor>(
			default_measurement_covariance_data)) {
		}

		KalmanFilter(
			const Eigen::Matrix<T, N, N> &state_transition_matrix,
			const Eigen::Matrix<T, K, N> &observation_matrix,
			const Eigen::Matrix<T, N, N> &process_covariance,
			const Eigen::Matrix<T, K, K> &default_measurement_covariance)
			: state_transition_matrix_(state_transition_matrix),
			observation_matrix_(observation_matrix),
			process_covariance_(process_covariance),
			default_measurement_covariance_(default_measurement_covariance) {
		}

		// Advances the system according to the current state estimate.
		void Step(State *state) const {
			state->mean = state_transition_matrix_ * state->mean;
			state->covariance = state_transition_matrix_ *
				state->covariance *
				state_transition_matrix_.transpose() +
				process_covariance_;
		}

		// Updates a state with a new measurement.
		void Update(const Eigen::Matrix<T, K, 1> &measurement_mean,
			const Eigen::Matrix<T, K, K> &measurement_covariance,
			State *state) const {
			// Calculate the innovation, which is a distribution over prediction error.
			Eigen::Matrix<T, K, 1> innovation_mean = measurement_mean -
				observation_matrix_ *
				state->mean;
			Eigen::Matrix<T, K, K> innovation_covariance =
				observation_matrix_ *
				state->covariance *
				observation_matrix_.transpose() +
				measurement_covariance;

			// Calculate the Kalman gain.
			Eigen::Matrix<T, 6, 2> kalman_gain = state->covariance *
				observation_matrix_.transpose() *
				innovation_covariance.inverse();

			// Update the state mean and covariance.
			state->mean += kalman_gain * innovation_mean;
			state->covariance = (Eigen::Matrix<T, N, N>::Identity() -
				kalman_gain * observation_matrix_) *
				state->covariance;
		}

		void Update(State *state,
			const Eigen::Matrix<T, K, 1> &measurement_mean) const {
			Update(state, measurement_mean, default_measurement_covariance_);
		}

	private:
		const Eigen::Matrix<T, N, N> state_transition_matrix_;
		const Eigen::Matrix<T, K, N> observation_matrix_;
		const Eigen::Matrix<T, N, N> process_covariance_;
		const Eigen::Matrix<T, K, K> default_measurement_covariance_;
	};
}
}  // namespace fblib



#endif // FBLIB_TRACKING_KALMAN_FILTER_H_
