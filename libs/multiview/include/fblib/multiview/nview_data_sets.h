#ifndef FBLIB_MULTIVIEW_NVIEW_DATA_SETS_H_
#define FBLIB_MULTIVIEW_NVIEW_DATA_SETS_H_

#include <vector>

#include "fblib/math/numeric.h"
using namespace fblib::math;

namespace fblib {
	namespace multiview{
		
		/** \brief	多视图的数据容器，包含相机和3d点及投影信息 */
		struct NViewDataSet {
			std::vector<Mat3> camera_matrix_;   //!< 相机内参(fx, fy, cx, cy).
			std::vector<Mat3> rotation_matrix_;   //!< 旋转矩阵R
			std::vector<Vec3> translation_vector_;   //!< 平移向量t
			std::vector<Vec3> camera_center_;   //!< 相机中心的世界坐标
			Mat3X point_3d_;          //!< 3D点.
			std::vector<Mat2X> projected_points_;  //!< 投影点，含噪声
			std::vector<Vecu>  projected_point_ids_;//!< 针对投影点的3d点的索引

			size_t actual_camera_num_;  //!< 实际相机的数目

			/**
			 * \brief	返回第i个相机的投影矩阵 P=K*[R|t]
			 *
			 * \param	i	相机的索引
			 *
			 * \return	投影矩阵
			 */
			Mat34 P(size_t i) const;

			/**	将点和相机信息导入到PLY文件中
			 */
			void ExportToPLY(const std::string &out_file_name) const;
		};

		/** \brief	多视图的数据配置项，包含相机内参及到原点距离及抖动误差*/
		struct NViewDatasetConfigurator
		{
			// 相机内参，焦距和中心点
			int _fx, _fy, _cx, _cy;

			// 相机随机位置参数
			double _dist;
			double _jitter_amount;

			NViewDatasetConfigurator(int fx = 1000, int fy = 1000,
				int cx = 500, int cy = 500,
				double distance = 1.5,
				double jitter_amount = 0.01);
		};

		/// 将相机放置一个圆的中点处
		NViewDataSet NRealisticCamerasRing(size_t nviews, size_t npoints,
			const NViewDatasetConfigurator
			config = NViewDatasetConfigurator());

		/// 将相机放置一个心形的中点处
		NViewDataSet NRealisticCamerasCardioid(size_t nviews, size_t npoints,
			const NViewDatasetConfigurator
			config = NViewDatasetConfigurator());
	}// namespace multiview
} // namespace fblib

#endif  // FBLIB_MULTIVIEW_NVIEW_DATA_SETS_H_
