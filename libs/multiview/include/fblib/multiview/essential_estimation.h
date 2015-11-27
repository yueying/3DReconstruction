#ifndef FBLIB_APPS_ROBUST_ESSENTIAL_H_
#define FBLIB_APPS_ROBUST_ESSENTIAL_H_

#include "fblib/camera/projection.h"
#include "fblib/feature/estimator_acransac.h"
#include "fblib/feature/estimator_acransac_kernel_adaptator.h"
#include "fblib/multiview/solver_essential_kernel.h"
#include "fblib/multiview/triangulation.h"

using namespace fblib::feature;
using fblib::camera::P_From_KRt;
using fblib::camera::Depth;

namespace fblib{
	namespace multiview{

		/**
		 * \brief	根据匹配点和相机内参，估计基础矩阵.
		 *
		 * \param	left_camera_intrinsic 	左相机的内参.
		 * \param	right_camera_intrinsic	右相机的内参.
		 * \param	left_points			  	左图像中的特征点
		 * \param	right_points		  	右图像中的特征点
		 * \param [in,out]	essential_matrix			  	If non-null, the p e.
		 * \param [in,out]	pvec_inliers  	If non-null, the pvec inliers.
		 * \param	left_image_size		  	左边图像的大小
		 * \param	right_image_size	  	右边图像的大小
		 * \param [in,out]	errorMax	  	If non-null, the error maximum.
		 * \param [in,out]	NFA			  	If non-null, the nfa.
		 * \param	precision			  	The precision.
		 *
		 * \return	true if it succeeds, false if it fails.
		 */

		bool robustEssential(
			const Mat3 &left_camera_intrinsic, const Mat3 &right_camera_intrinsic,
			const Mat &left_points, const Mat &right_points,
			Mat3 *essential_matrix,
			std::vector<size_t> * pvec_inliers,
			const std::pair<size_t, size_t> &left_image_size,
			const std::pair<size_t, size_t> &right_image_size,
			double * errorMax,
			double * NFA,
			double precision = std::numeric_limits<double>::infinity())
		{
			assert(pvec_inliers != NULL);
			assert(essential_matrix != NULL);

			// 通过5点算法求解本质矩阵essential_matrix
			typedef fblib::multiview::essential::FivePointKernel SolverType;
			// Define the AContrario adaptor
			typedef ACKernelAdaptorEssential <
				SolverType,
				fblib::multiview::fundamental::EpipolarDistanceError,
				UnnormalizerT,
				Mat3 >
				KernelType;

			KernelType kernel(left_points, left_image_size.first, left_image_size.second,
				right_points, right_image_size.first, right_image_size.second, left_camera_intrinsic, right_camera_intrinsic);

			// Robustly estimation of the Essential matrix and it's precision
			std::pair<double, double> acransac_out = ACRANSAC(kernel, *pvec_inliers,
				4096, essential_matrix, precision, true);
			*errorMax = acransac_out.first;
			*NFA = acransac_out.second;

			return pvec_inliers->size() > 2.5 * SolverType::MINIMUM_SAMPLES;
		}

		/**
		 * \brief	通过本质矩阵计算外参，外参的可能性有四种，计算点在相机前的点
		 *
		 * \param	left_camera_intrinsic	  	左相机内参
		 * \param	right_camera_intrinsic	  	右相机内参
		 * \param	left_points				  	左图像对应点
		 * \param	right_points			  	有图像对应点
		 * \param	essential_matrix		  	本质矩阵
		 * \param	vec_inliers				  	对应点的索引
		 * \param [in,out]	rotation_matrix   	返回外参的旋转矩阵
		 * \param [in,out]	translation_vector	返回外参的平移向量
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		bool estimateRtFromE(const Mat3 &left_camera_intrinsic, const Mat3 &right_camera_intrinsic,
			const Mat &left_points, const Mat &right_points,
			const Mat3 &essential_matrix, const std::vector<size_t> &vec_inliers,
			Mat3 *rotation_matrix, Vec3 *translation_vector)
		{
			bool is_ok = false;

			// 累加器，用于找到最佳的解决方案
			std::vector<size_t> f(4, 0);

			std::vector<Mat3> vec_essential; // 本质矩阵
			std::vector<Mat3> vec_rotation;  // 旋转矩阵
			std::vector<Vec3> vec_translation;  // 平移向量

			vec_essential.push_back(essential_matrix);
			// 从本质矩阵中恢复相机的外参
			MotionFromEssential(essential_matrix, &vec_rotation, &vec_translation);

			// 对所有的点测试本质矩阵分解的4种可能性
			assert(vec_rotation.size() == 4);
			assert(vec_translation.size() == 4);

			Mat34 P1, P2;
			Mat3 R1 = Mat3::Identity();
			Vec3 t1 = Vec3::Zero();
			P_From_KRt(left_camera_intrinsic, R1, t1, &P1);

			for (int i = 0; i < 4; ++i) {
				const Mat3 &R2 = vec_rotation[i];
				const Vec3 &t2 = vec_translation[i];
				P_From_KRt(right_camera_intrinsic, R2, t2, &P2);
				Vec3 X;

				for (size_t k = 0; k < vec_inliers.size(); ++k)
				{
					const Vec2 & x1_ = left_points.col(vec_inliers[k]);
					const Vec2 & x2_ = right_points.col(vec_inliers[k]);
					TriangulateDLT(P1, x1_, P2, x2_, &X);
					// 测试点是否在两个相机的前面
					if (Depth(R1, t1, X) > 0 && Depth(R2, t2, X) > 0) {
						++f[i];
					}
				}
			}
			// 对结果进行分析
			std::cout << "\translation_vector Number of points in front of both cameras:"
				<< f[0] << " " << f[1] << " " << f[2] << " " << f[3] << std::endl;
			std::vector<size_t>::iterator iter = max_element(f.begin(), f.end());
			if (*iter != 0)
			{
				size_t index = std::distance(f.begin(), iter);
				(*rotation_matrix) = vec_rotation[index];
				(*translation_vector) = vec_translation[index];
				is_ok = true;
			}
			else  {
				std::cerr << std::endl << "/!\\There is no right solution,"
					<< " probably intermediate results are not correct or no points"
					<< " in front of both cameras" << std::endl;
				is_ok = false;
			}
			return is_ok;
		}
	}// namespace multiview
} // namespace fblib

#endif // FBLIB_APPS_ROBUST_ESSENTIAL_H_
