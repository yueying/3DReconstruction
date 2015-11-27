#ifndef FBLIB_FEATURE_ESTIMATOR_ACRANSAC_KERNEL_ADAPTATOR_H_
#define FBLIB_FEATURE_ESTIMATOR_ACRANSAC_KERNEL_ADAPTATOR_H_

// Here a collection of A contrario Kernel adaptor.
//ACKernelAdaptor
//  For general two view estimation (affine, homography, fundamental)
//ACKernelAdaptorResection
//  For pose/resection estimation
//ACKernelAdaptorResection_K
//  For pose/resection with known camera intrinsic
//ACKernelAdaptorEssential
//  For essential matrix estimation
//
// Mainly it add correct data normalization and define the function required
//  by the generic ACRANSAC routine.
//
#include "fblib/math/numeric.h"
#include "fblib/multiview/conditioning.h"
using namespace fblib::math;
using namespace fblib::multiview;
namespace fblib {
	namespace feature{

		/// Two view Kernel adaptator for the A contrario model estimator
		/// Handle data normalization and compute the corresponding logalpha 0
		///  that depends of the error model (point to line, or point to point)
		/// This kernel adaptor is working for affine, homography, fundamental matrix
		///  estimation.
		template <typename SolverArg,
			typename ErrorArg,
			typename UnnormalizerArg,
			typename ModelArg = Mat3>
		class ACKernelAdaptor
		{
		public:
			typedef SolverArg Solver;
			typedef ModelArg  Model;
			typedef ErrorArg ErrorT;

			ACKernelAdaptor(const Mat &left_points, int left_image_width, int left_image_hight,
				const Mat &right_points, int right_image_width, int right_image_hight, bool is_point_to_line = true)
				: left_points_(left_points.rows(), left_points.cols()), x2_(right_points.rows(), right_points.cols()),
				N1_(3, 3), N2_(3, 3), logalpha0_(0.0), is_point_to_line_(is_point_to_line)
			{
				assert(2 == left_points_.rows());
				assert(left_points_.rows() == x2_.rows());
				assert(left_points_.cols() == x2_.cols());

				NormalizePoints(left_points, &left_points_, &N1_, left_image_width, left_image_hight);
				NormalizePoints(right_points, &x2_, &N2_, right_image_width, right_image_hight);

				// LogAlpha0 is used to make error data scale invariant
				if (is_point_to_line)  {
					// Ratio of containing diagonal image rectangle over image area
					double D = sqrt(right_image_width*(double)right_image_width + right_image_hight*(double)right_image_hight); // diameter
					double A = right_image_width*(double)right_image_hight; // area
					logalpha0_ = log10(2.0*D / A / N2_(0, 0));
				}
				else  {
					// ratio of area : unit circle over image area
					logalpha0_ = log10(M_PI / (right_image_width*(double)right_image_hight) / (N2_(0, 0)*N2_(0, 0)));
				}
			}

			enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
			enum { MAX_MODELS = Solver::MAX_MODELS };

			void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const {
				Mat left_points = extractColumns(left_points_, samples);
				Mat right_points = extractColumns(x2_, samples);
				Solver::Solve(left_points, right_points, models);
			}

			double Error(size_t sample, const Model &model) const {
				return ErrorT::Error(model, left_points_.col(sample), x2_.col(sample));
			}

			size_t NumSamples() const {
				return static_cast<size_t>(left_points_.cols());
			}

			void Unnormalize(Model * model) const {
				// Unnormalize model from the computed conditioning.
				UnnormalizerArg::Unnormalize(N1_, N2_, &(*model));
			}

			double logalpha0() const { return logalpha0_; }

			double multError() const { return (is_point_to_line_) ? 0.5 : 1.0; }

			Mat3 normalizer1() const { return N1_; }
			Mat3 normalizer2() const { return N2_; }
			double unormalizeError(double val) const { return sqrt(val) / N2_(0, 0); }

		private:
			Mat left_points_, x2_;       // Normalized input data
			Mat3 N1_, N2_;      // Matrix used to normalize data
			double logalpha0_; // Alpha0 is used to make the error adaptive to the image size
			bool is_point_to_line_;// Store if error model is pointToLine or point to point
		};

		struct UnnormalizerResection {
			static void Unnormalize(const Mat &T, const Mat &U, Mat34 *P){
				*P = T.inverse() * (*P);
			}
		};

		/// Pose/Resection Kernel adaptator for the A contrario model estimator
		template <typename SolverArg,
			typename ErrorArg,
			typename UnnormalizerArg,
			typename ModelArg = Mat34>
		class ACKernelAdaptorResection
		{
		public:
			typedef SolverArg Solver;
			typedef ModelArg  Model;
			typedef ErrorArg ErrorT;

			ACKernelAdaptorResection(const Mat &x2d, int w, int h, const Mat &x3D)
				: x2d_(x2d.rows(), x2d.cols()), right_points_(x3D),
				N1_(3, 3), logalpha0_(log10(M_PI))
			{
				assert(2 == x2d_.rows());
				assert(3 == right_points_.rows());
				assert(x2d_.cols() == right_points_.cols());

				NormalizePoints(x2d, &x2d_, &N1_, w, h);
			}

			enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
			enum { MAX_MODELS = Solver::MAX_MODELS };

			void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const {
				Mat left_points = extractColumns(x2d_, samples);
				Mat right_points = extractColumns(right_points_, samples);
				Solver::Solve(left_points, right_points, models);
			}

			double Error(int sample, const Model &model) const {
				return ErrorT::Error(model, x2d_.col(sample), right_points_.col(sample));
			}

			size_t NumSamples() const { return x2d_.cols(); }

			void Unnormalize(Model * model) const {
				// Unnormalize model from the computed conditioning.
				UnnormalizerArg::Unnormalize(N1_, Mat3::Identity(), &(*model));
			}

			double logalpha0() const { return logalpha0_; }
			double multError() const { return 1.0; } // point to point error
			Mat3 normalizer1() const { return N1_; }
			Mat3 normalizer2() const { return Mat3::Identity(); }
			double unormalizeError(double val) const { return sqrt(val) / N1_(0, 0); }

		private:
			Mat x2d_, right_points_;
			Mat3 N1_;      // Matrix used to normalize data
			double logalpha0_; // Alpha0 is used to make the error adaptive to the image size
		};

		/// Pose/Resection Kernel adaptator for the A contrario model estimator with
		///  known Intrinsic
		template <typename SolverArg,
			typename ErrorArg,
			typename UnnormalizerArg,
			typename ModelArg = Mat34>
		class ACKernelAdaptorResection_K
		{
		public:
			typedef SolverArg Solver;
			typedef ModelArg  Model;
			typedef ErrorArg ErrorT;

			ACKernelAdaptorResection_K(const Mat &x2d, const Mat &x3D, const Mat3 & K)
				: x2d_(x2d.rows(), x2d.cols()), right_points_(x3D),
				N1_(K.inverse()),
				logalpha0_(log10(M_PI)), K_(K)
			{
				assert(2 == x2d_.rows());
				assert(3 == right_points_.rows());
				assert(x2d_.cols() == right_points_.cols());

				// Normalize points by inverse(K)
				applyTransformationToPoints(x2d, N1_, &x2d_);
			}

			enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
			enum { MAX_MODELS = Solver::MAX_MODELS };

			void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const {
				Mat left_points = extractColumns(x2d_, samples);
				Mat right_points = extractColumns(right_points_, samples);
				Solver::Solve(left_points, right_points, models);
			}

			double Error(size_t sample, const Model &model) const {
				return ErrorT::Error(model, x2d_.col(sample), right_points_.col(sample));
			}

			size_t NumSamples() const { return x2d_.cols(); }

			void Unnormalize(Model * model) const {
				// Unnormalize model from the computed conditioning.
				(*model) = K_ * (*model);
			}

			double logalpha0() const { return logalpha0_; }
			double multError() const { return 1.0; } // point to point error
			Mat3 normalizer1() const { return N1_; }
			Mat3 normalizer2() const { return Mat3::Identity(); }
			double unormalizeError(double val) const { return sqrt(val) / N1_(0, 0); }

		private:
			Mat x2d_, right_points_;
			Mat3 N1_;      // Matrix used to normalize data
			double logalpha0_; // Alpha0 is used to make the error adaptive to the image size
			Mat3 K_;            // Intrinsic camera parameter
		};

		/// Essential matrix Kernel adaptator for the A contrario model estimator
		template <typename SolverArg,
			typename ErrorArg,
			typename UnnormalizerArg,
			typename ModelArg = Mat3>
		class ACKernelAdaptorEssential
		{
		public:
			typedef SolverArg Solver;
			typedef ModelArg  Model;
			typedef ErrorArg ErrorT;

			ACKernelAdaptorEssential(
				const Mat &left_points, int left_image_width, int left_image_hight,
				const Mat &right_points, int right_image_width, int right_image_hight,
				const Mat3 &left_camera_intrinsic, const Mat3 &right_camera_intrinsic)
				: left_points_(left_points), right_points_(right_points),
				N1_(Mat3::Identity()), N2_(Mat3::Identity()), logalpha0_(0.0),
				left_camera_intrinsic_(left_camera_intrinsic), right_camera_intrinsic_(right_camera_intrinsic)
			{
				// 确保输入的是点，以及对应点数应保持一致
				assert(2 == left_points_.rows());
				assert(left_points_.rows() == right_points_.rows());
				assert(left_points_.cols() == right_points_.cols());
				// 将图像像素坐标转相机成像平面的位置坐标
				applyTransformationToPoints(left_points_, left_camera_intrinsic_.inverse(), &left_camera_plane_points_);
				applyTransformationToPoints(right_points_, right_camera_intrinsic_.inverse(), &right_camera_plane_points_);

				//Point to line probability (line is the epipolar line)
				double D = sqrt(right_image_width*(double)right_image_width + right_image_hight*(double)right_image_hight); // 直径
				double A = right_image_width*(double)right_image_hight; // 面积
				logalpha0_ = log10(2.0*D / A * .5);
			}

			enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
			enum { MAX_MODELS = Solver::MAX_MODELS };

			void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const {
				Mat left_points = extractColumns(left_camera_plane_points_, samples);
				Mat right_points = extractColumns(right_camera_plane_points_, samples);
				Solver::Solve(left_points, right_points, models);
			}

			double Error(size_t sample, const Model &model) const {
				Mat3 fundamental_matrix;
				FundamentalFromEssential(model, left_camera_intrinsic_, right_camera_intrinsic_, &fundamental_matrix);
				return ErrorT::Error(fundamental_matrix, this->left_points_.col(sample), this->right_points_.col(sample));
			}

			size_t NumSamples() const { return left_points_.cols(); }
			void Unnormalize(Model * model) const {}
			double logalpha0() const { return logalpha0_; }
			double multError() const { return 0.5; } // point to line error
			Mat3 normalizer1() const { return N1_; }
			Mat3 normalizer2() const { return N2_; }
			double unormalizeError(double val) const { return val; }

		private:
			Mat left_points_, right_points_, left_camera_plane_points_, right_camera_plane_points_; //!< 图像点和相机平面点
			Mat3 N1_, N2_;      // Matrix used to normalize data
			double logalpha0_; // Alpha0 is used to make the error adaptive to the image size
			Mat3 left_camera_intrinsic_, right_camera_intrinsic_;      //!< 左右相机的内外参
		};

	} // namespace feature
} // namespace fblib

#endif // FBLIB_FEATURE_ESTIMATOR_ACRANSAC_KERNEL_ADAPTATOR_H_
