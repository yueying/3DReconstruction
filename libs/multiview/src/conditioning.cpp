#include "multiview_precomp.h"
#include "fblib/multiview/conditioning.h"

using namespace fblib::math;

namespace fblib {
	namespace multiview{
		// HZ 4.4.4 pag.109
		void PreconditionerFromPoints(const Mat &points, Mat3 *T) {
			Vec mean, variance;
			MeanAndVarianceAlongRows(points, &mean, &variance);

			double xfactor = sqrt(2.0 / variance(0));
			double yfactor = sqrt(2.0 / variance(1));

			// If variance is equal to 0.0 set scaling factor to identity.
			// -> Else it will provide nan value (because division by 0).
			if (variance(0) < 1e-8)
				xfactor = mean(0) = 1.0;
			if (variance(1) < 1e-8)
				yfactor = mean(1) = 1.0;

			*T << xfactor, 0, -xfactor * mean(0),
				0, yfactor, -yfactor * mean(1),
				0, 0, 1;
		}

		// HZ 4.4.4 pag.107: Point conditioning (isotropic)
		void IsotropicPreconditionerFromPoints(const Mat &points, Mat3 *T) {
			Vec mean, variance;
			MeanAndVarianceAlongRows(points, &mean, &variance);

			double var_norm = variance.norm();
			double factor = sqrt(2.0 / var_norm);

			// If variance is equal to 0.0 set scaling factor to identity.
			// -> Else it will provide nan value (because division by 0).
			if (var_norm < 1e-8) {
				factor = 1.0;
				mean.setOnes();
			}

			*T << factor, 0, -factor * mean(0),
				0, factor, -factor * mean(1),
				0, 0, 1;
		}

		void PreconditionerFromPoints(int width, int height, Mat3 *T) {
			// Build the normalization matrix
			double dNorm = 1.0 / sqrt(static_cast<double>(width*height));

			(*T) = Mat3::Identity();
			(*T)(0, 0) = (*T)(1, 1) = dNorm;
			(*T)(0, 2) = -.5f*width*dNorm;
			(*T)(1, 2) = -.5*height*dNorm;
		}

		void applyTransformationToPoints(const Mat &points, const Mat3 &T, Mat *transformed_points) 
		{
			const Mat::Index n = points.cols();//Mat::Index 索引的类型,点的个数
			transformed_points->resize(2, n);//设置变换之后的矩阵
			for (Mat::Index i = 0; i < n; ++i) {
				Vec3 in, out;
				in << points(0, i), points(1, i), 1;
				out = T * in;
				(*transformed_points)(0, i) = out(0) / out(2);
				(*transformed_points)(1, i) = out(1) / out(2);
			}
		}

		void NormalizePoints(const Mat &points,
			Mat *normalized_points,
			Mat3 *T,
			int width,
			int height) {
			PreconditionerFromPoints(width, height, T);
			applyTransformationToPoints(points, *T, normalized_points);
		}


		void NormalizePoints(const Mat &points,
			Mat *normalized_points,
			Mat3 *T) {
			PreconditionerFromPoints(points, T);
			applyTransformationToPoints(points, *T, normalized_points);
		}

		void NormalizeIsotropicPoints(const Mat &points,
			Mat *normalized_points,
			Mat3 *T) {
			IsotropicPreconditionerFromPoints(points, T);
			applyTransformationToPoints(points, *T, normalized_points);
		}

		// Denormalize the results. See HZ page 109.
		void UnnormalizerT::Unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H)  {
			*H = T2.transpose() * (*H) * T1;
		}

		// Denormalize the results. See HZ page 109.
		void UnnormalizerI::Unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H)  {
			*H = T2.inverse() * (*H) * T1;
		}
	}// namespace multiview
} // namespace fblib
