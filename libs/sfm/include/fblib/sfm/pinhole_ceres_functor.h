#ifndef FBLIB_SFM_PINHOLE_CERES_FUNCTOR_H
#define FBLIB_SFM_PINHOLE_CERES_FUNCTOR_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace fblib{
	namespace sfm{

		///定义最小化投影误差的函数子(||x^j_i -P_{ ij }(X_j) || ) 表示一个三维点X_j投影到一个图像i上与观察值x^j_i的比较

		/**
		 * \brief	计算投影误差 residual = observed -euclidean( f * [R|t] X)
		 *
		 * \tparam	T	参数类型
		 * \param	cam_R				 	相机外参R
		 * \param	cam_t				 	相机外参t
		 * \param	cam_f				 	相机内参f
		 * \param	pos_3dpoint			 	观察到的三维点
		 * \param	pos_2dpoint			 	图像平面上观察到的二维点
		 * \param [in,out]	out_residuals	x和y轴上的误差
		 */
		template <typename T>
		void ComputeResidual(
			const T* const cam_R,
			const T* const cam_t,
			const T* const cam_f,
			const T* const pos_3dpoint,
			const double* pos_2dpoint,
			T* out_residuals)
		{
			T pos_proj[3];

			// 应用相机外参旋转
			ceres::AngleAxisRotatePoint(cam_R, pos_3dpoint, pos_proj);

			// 应用相机的平移
			pos_proj[0] += cam_t[0];
			pos_proj[1] += cam_t[1];
			pos_proj[2] += cam_t[2];

			// 将点齐次坐标转到欧式坐标
			T xe = pos_proj[0] / pos_proj[2];
			T ye = pos_proj[1] / pos_proj[2];

			// 应用相机内参
			const T& focal = cam_f[0];
			T predicted_x = focal * xe;
			T predicted_y = focal * ye;

			// 计算返回观察与预测之间的误差
			out_residuals[0] = predicted_x - T(pos_2dpoint[0]);
			out_residuals[1] = predicted_y - T(pos_2dpoint[1]);
		}

		/**
		 * \brief	定义一个Ceres函数子用于定义针孔相机模型，用于计算投影误差
		 */
		struct ErrorFuncRefineCamera3DPointsFocal
		{
			ErrorFuncRefineCamera3DPointsFocal(const double* const pos_2dpoint)
			{
				m_pos_2dpoint[0] = pos_2dpoint[0];
				m_pos_2dpoint[1] = pos_2dpoint[1];
			}

			/**
			 * \brief	重载()操作符
			 * \tparam	T	
			 * \param	cam_f				 	相机内参焦距
			 * \param	cam_Rt				 	相机外参，为6自由度，6个参数
			 * \param	pos_3dpoint			 	观察到的三维点
			 * \param [in,out]	out_residuals	投影误差
			 *
			 */
			template <typename T>
			bool operator()(
				const T* const cam_f,
				const T* const cam_Rt,
				const T* const pos_3dpoint,
				T* out_residuals) const
			{
				ComputeResidual(
					cam_Rt, // => cam_R
					&cam_Rt[3], // => cam_t
					cam_f,
					pos_3dpoint,
					m_pos_2dpoint,
					out_residuals);

				return true;
			}

			double m_pos_2dpoint[2]; //!< 二维观测点
		};

		/**
		* \brief	定义一个Ceres函数子用于定义针孔相机模型，用于计算投影误差
		*/
		struct ErrorFuncRefineCamera3DPointsPinhole
		{
			ErrorFuncRefineCamera3DPointsPinhole(const double* const pos_2dpoint)
			{
				m_pos_2dpoint[0] = pos_2dpoint[0];
				m_pos_2dpoint[1] = pos_2dpoint[1];
			}

			/**
			 * \brief \a cam_Rtf 参数使用7参数[R;t;f]
			 */
			template <typename T>
			bool operator()(
				const T* const cam_Rtf, // [R;t;f]
				const T* const pos_3dpoint,
				T* out_residuals) const
			{
				ComputeResidual(
					cam_Rtf, // => cam_R
					&cam_Rtf[3], // => cam_t
					&cam_Rtf[6], // => cam_f
					pos_3dpoint,
					m_pos_2dpoint,
					out_residuals);

				return true;
			}

			double m_pos_2dpoint[2]; //!< 二维观测点
		};

		/**
		 * \brief	定义一个Ceres函数子用于定义针孔相机模型，用于计算投影误差
		 * 			给出观察到的二维和三维点
		 */
		struct ErrorFuncRefineCamera
		{
			ErrorFuncRefineCamera(const double* const pos_2dpoint, const double* const pos_3dpoint)
			{
				m_pos_2dpoint[0] = pos_2dpoint[0];
				m_pos_2dpoint[1] = pos_2dpoint[1];

				m_pos_3dpoint[0] = pos_3dpoint[0];
				m_pos_3dpoint[1] = pos_3dpoint[1];
				m_pos_3dpoint[2] = pos_3dpoint[2];
			}

			/**
			*  \brief \a cam_Rtf 参数使用7参数[R;t;f]
			*/
			template <typename T>
			bool operator()(
				const T* const cam_Rtf, // [R;t;f]
				T* out_residuals) const
			{
				T pos_3dpoint[3];
				pos_3dpoint[0] = T(m_pos_3dpoint[0]);
				pos_3dpoint[1] = T(m_pos_3dpoint[1]);
				pos_3dpoint[2] = T(m_pos_3dpoint[2]);

				ComputeResidual(
					cam_Rtf, // => cam_R
					&cam_Rtf[3], // => cam_t
					&cam_Rtf[6], // => cam_f
					pos_3dpoint,
					m_pos_2dpoint,
					out_residuals);

				return true;
			}

			double m_pos_2dpoint[2]; //!< 二维观测点
			double m_pos_3dpoint[3]; //!< 三维观测点
		};

	} // namespace sfm
} // namespace fblib

#endif

