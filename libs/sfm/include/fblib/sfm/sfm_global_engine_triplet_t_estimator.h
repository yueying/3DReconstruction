#ifndef FBLIB_SFM_GLOBAL_SFM_ENGINE_TRIPLET_T_ESTIMATOR_H
#define FBLIB_SFM_GLOBAL_SFM_ENGINE_TRIPLET_T_ESTIMATOR_H

#include "fblib/math/numeric.h"

#include "fblib/sfm/linear_programming_interface.h"
#include "fblib/sfm/linear_programming_osi.h"

#include "fblib/sfm/bisection_linear_programming.h"
#include "fblib/sfm/tijs_and_xis_from_xi_ri.h"

#include "fblib/feature/estimator_acransac.h"


namespace fblib{
	namespace sfm{

		/// A trifocal tensor seen as 3 projective cameras
		struct TrifocalTensorModel {
			Mat34 P1, P2, P3;

			static double Error(const TrifocalTensorModel & t, const Vec2 & pt1, const Vec2 & pt2, const Vec2 & pt3)
			{
				// Triangulate and return the reprojection error
				Triangulation triangulationObj;
				triangulationObj.add(t.P1, pt1);
				triangulationObj.add(t.P2, pt2);
				triangulationObj.add(t.P3, pt3);
				const Vec3 X = triangulationObj.compute();

				//- Return max error as a test
				double pt1ReProj = (Project(t.P1, X) - pt1).squaredNorm();
				double pt2ReProj = (Project(t.P2, X) - pt2).squaredNorm();
				double pt3ReProj = (Project(t.P3, X) - pt3).squaredNorm();

				return std::max(pt1ReProj, std::max(pt2ReProj, pt3ReProj));
			}
		};

		struct tisXisTrifocalSolver {
			enum { MINIMUM_SAMPLES = 4 };
			enum { MAX_MODELS = 1 };
			// Solve the computation of the tensor.
			static void Solve(
				const Mat &pt0, const Mat & pt1, const Mat & pt2,
				const std::vector<Mat3> & vec_KR, const Mat3 & K, std::vector<TrifocalTensorModel> *P)
			{
				//Build the megaMatMatrix
				const int n_obs = pt0.cols();
				Mat4X megaMat(4, n_obs * 3);
				{
					size_t cpt = 0;
					for (size_t i = 0; i < n_obs; ++i) {

						megaMat.col(cpt) << pt0.col(i)(0), pt0.col(i)(1), i, 0;
						++cpt;

						megaMat.col(cpt) << pt1.col(i)(0), pt1.col(i)(1), i, 1;
						++cpt;

						megaMat.col(cpt) << pt2.col(i)(0), pt2.col(i)(1), i, 2;
						++cpt;
					}
				}
				//-- Solve the LInfinity translation and structure from Rotation and points data.
				std::vector<double> vec_solution((3 + MINIMUM_SAMPLES) * 3);

				OSI_CLP_SolverWrapper LPsolver(static_cast<int>(vec_solution.size()));


				Translation_Structure_L1_ConstraintBuilder constraint_builder(vec_KR, megaMat);
				double gamma;
				if (BisectionLinearProgramming<Translation_Structure_L1_ConstraintBuilder, LP_Constraints_Sparse>(
					LPsolver,
					constraint_builder,
					&vec_solution,
					.5 / K(0, 0),//admissibleResidual,
					0.0, 1e-8, 2, &gamma, false))
				{
					std::vector<Vec3> vec_tis(3);
					vec_tis[0] = Vec3(vec_solution[0], vec_solution[1], vec_solution[2]);
					vec_tis[1] = Vec3(vec_solution[3], vec_solution[4], vec_solution[5]);
					vec_tis[2] = Vec3(vec_solution[6], vec_solution[7], vec_solution[8]);

					TrifocalTensorModel PTemp;
					PTemp.P1 = HStack(vec_KR[0], vec_tis[0]);
					PTemp.P2 = HStack(vec_KR[1], vec_tis[1]);
					PTemp.P3 = HStack(vec_KR[2], vec_tis[2]);

					P->push_back(PTemp);
				}
			}

			// Compute the residual of reprojections
			static double Error(const TrifocalTensorModel & Tensor, const Vec2 & pt0, const Vec2 & pt1, const Vec2 & pt2)
			{
				return TrifocalTensorModel::Error(Tensor, pt0, pt1, pt2);
			}
		};

		template <typename SolverArg,
			typename ErrorArg,
			typename ModelArg>
		class TrifocalKernel_ACRansac_N_tisXis
		{
		public:
			typedef SolverArg Solver;
			typedef ModelArg  Model;


			TrifocalKernel_ACRansac_N_tisXis(const Mat & x1, const Mat & x2, const Mat & x3,
				int w, int h, const std::vector<Mat3> & vec_KRi, const Mat3 & K)
				: x1_(x1), x2_(x2), x3_(x3), logalpha0_(0.0), vec_KR_(vec_KRi), K_(K)
			{
				// Normalize points by inverse(K)

				N_ = K_.inverse();
				applyTransformationToPoints(x1_, N_, &x1n_);
				applyTransformationToPoints(x2_, N_, &x2n_);
				applyTransformationToPoints(x3_, N_, &x3n_);

				vec_KR_[0] = N_ * vec_KR_[0];
				vec_KR_[1] = N_ * vec_KR_[1];
				vec_KR_[2] = N_ * vec_KR_[2];

				logalpha0_ = log10(M_PI);
			}

			enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
			enum { MAX_MODELS = Solver::MAX_MODELS };

			void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const {

				// Create a model from the points
				Solver::Solve(
					extractColumns(x1n_, samples),
					extractColumns(x2n_, samples),
					extractColumns(x3n_, samples),
					vec_KR_, K_, models);
			}

			double Error(size_t sample, const Model &model) const {
				return ErrorArg::Error(model, x1n_.col(sample), x2n_.col(sample), x3n_.col(sample));
			}

			size_t NumSamples() const {
				return x1n_.cols();
			}

			void Unnormalize(Model * model) const {
				// Unnormalize model from the computed conditioning.
				model->P1 = K_ * model->P1;
				model->P2 = K_ * model->P2;
				model->P3 = K_ * model->P3;
			}

			double logalpha0() const { return logalpha0_; }

			double multError() const { return 1.0; }

			Mat3 normalizer1() const { return N_; }
			Mat3 normalizer2() const { return Mat3::Identity(); }
			double unormalizeError(double val) const { return sqrt(val) / N_(0, 0); }

		private:
			const Mat & x1_, &x2_, &x3_;
			Mat x1n_, x2n_, x3n_;
			Mat3 N_;
			double logalpha0_;
			std::vector<Mat3> vec_KR_;
			Mat3 K_;
		};
	}
} // namespace fblib

#endif // FBLIB_SFM_GLOBAL_SFM_ENGINE_TRIPLET_T_ESTIMATOR_H
