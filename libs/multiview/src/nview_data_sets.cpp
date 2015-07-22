#include "multiview_precomp.h"
#include "fblib/multiview/nview_data_sets.h"

#include <cmath>
#include <fstream>

#include "fblib/math/numeric.h"
#include "fblib/multiview/projection.h"

namespace fblib {
	namespace multiview{

		NViewDatasetConfigurator::NViewDatasetConfigurator(int fx, int fy,
			int cx, int cy, double distance, double jitter_amount) :
			_fx(fx), _fy(fy), _cx(cx), _cy(cy), _dist(distance),
			_jitter_amount(jitter_amount)
		{}

		NViewDataSet NRealisticCamerasRing(size_t nviews, size_t npoints,
			const NViewDatasetConfigurator config)
		{
			// 设置相机参数
			NViewDataSet d;
			d.actual_camera_num_ = nviews;
			d.camera_matrix_.resize(nviews);
			d.rotation_matrix_.resize(nviews);
			d.translation_vector_.resize(nviews);
			d.camera_center_.resize(nviews);
			d.projected_points_.resize(nviews);
			d.projected_point_ids_.resize(nviews);

			d.point_3d_.resize(3, npoints);
			d.point_3d_.setRandom();
			d.point_3d_ *= 0.6;

			Vecu all_point_ids(npoints);
			for (size_t j = 0; j < npoints; ++j)
				all_point_ids[j] = j;

			for (size_t i = 0; i < nviews; ++i) {
				Vec3 camera_center, t, jitter, lookdir;

				const double theta = i * 2 * M_PI / nviews;
				// 圆的方程式
				camera_center << sin(theta), 0.0, cos(theta); // Y axis UP
				camera_center *= config._dist;
				d.camera_center_[i] = camera_center;

				jitter.setRandom();
				jitter *= config._jitter_amount / camera_center.norm();
				lookdir = -camera_center + jitter;

				d.camera_matrix_[i] << config._fx, 0, config._cx,
					0, config._fy, config._cy,
					0, 0, 1;
				d.rotation_matrix_[i] = LookAt(lookdir);  // Y axis UP
				d.translation_vector_[i] = -d.rotation_matrix_[i] * camera_center; // [t]=[-RC] Cf HZ.
				d.projected_points_[i] = Project(d.P(i), d.point_3d_);
				d.projected_point_ids_[i] = all_point_ids;
			}
			return d;
		}

		Mat34 NViewDataSet::P(size_t i)const {
			assert(i < actual_camera_num_);
			Mat34 P;
			P_From_KRt(camera_matrix_[i], rotation_matrix_[i], translation_vector_[i], &P);
			return P;
		}

		void NViewDataSet::ExportToPLY(
			const std::string & out_file_name)const {
			std::ofstream outfile;
			outfile.open(out_file_name.c_str(), std::ios_base::out);
			if (outfile.is_open()) {
				outfile << "ply"
					<< std::endl << "format ascii 1.0"
					<< std::endl << "comment NViewDataSet export"
					<< std::endl << "comment It shows 3D point structure and cameras"
					<< "+ camera looking direction"
					<< std::endl << "element vertex " << point_3d_.cols() + translation_vector_.size() * 2
					<< std::endl << "property float x"
					<< std::endl << "property float y"
					<< std::endl << "property float z"
					<< std::endl << "property uchar red"
					<< std::endl << "property uchar green"
					<< std::endl << "property uchar blue"
					<< std::endl << "end_header" << std::endl;

				// 导出3d点云数据
				for (Mat3X::Index i = 0; i < point_3d_.cols(); ++i) {
					// 导出点的位置和点的颜色
					outfile << point_3d_.col(i).transpose()
						<< " " << "255 255 255" << std::endl;
				}

				// 导出3d相机位置 t = -RC
				for (size_t i = 0; i < translation_vector_.size(); ++i) {
					// 导出相机的位置和相机的颜色
					outfile << (-rotation_matrix_[i].transpose()*translation_vector_[i]).transpose()
						<< " " << "0 255 0" << std::endl;
				}
				for (size_t i = 0; i < translation_vector_.size(); ++i) {
					Vec3 test;
					test << 0, 0, 0.4;
					// 导出相机法线
					outfile << ((-rotation_matrix_[i].transpose()*translation_vector_[i]) +
						(rotation_matrix_[i].transpose()*test)).transpose()
						<< " " << "255 0 0" << std::endl;
				}
				outfile.close();
			}
		}

		NViewDataSet NRealisticCamerasCardioid(size_t nviews, size_t npoints,
			const NViewDatasetConfigurator config)
		{
			// 设置相机参数
			NViewDataSet d;
			d.actual_camera_num_ = nviews;
			d.camera_matrix_.resize(nviews);
			d.rotation_matrix_.resize(nviews);
			d.translation_vector_.resize(nviews);
			d.camera_center_.resize(nviews);
			d.projected_points_.resize(nviews);
			d.projected_point_ids_.resize(nviews);

			d.point_3d_.resize(3, npoints);
			d.point_3d_.setRandom();
			d.point_3d_ *= 0.6;

			Vecu all_point_ids(npoints);
			for (size_t j = 0; j < npoints; ++j)
				all_point_ids[j] = j;

			for (size_t i = 0; i < nviews; ++i) {
				Vec3 camera_center, t, jitter, lookdir;

				const double theta = i * 2 * M_PI / nviews;
				// 心形方程式，确定中点
				camera_center <<
					2 * sin(theta) - (sin(2 * theta)),
					0.0,
					2 * cos(theta) - (cos(2 * theta)); // Y axis UP
				camera_center *= config._dist;
				d.camera_center_[i] = camera_center;

				jitter.setRandom();
				jitter *= config._jitter_amount / camera_center.norm();
				lookdir = -camera_center + jitter;

				d.camera_matrix_[i] << config._fx, 0, config._cx,
					0, config._fy, config._cy,
					0, 0, 1;
				d.rotation_matrix_[i] = LookAt(lookdir);  // Y axis UP
				d.translation_vector_[i] = -d.rotation_matrix_[i] * camera_center; // [t]=[-RC] Cf HZ.
				d.projected_points_[i] = Project(d.P(i), d.point_3d_);
				d.projected_point_ids_[i] = all_point_ids;
			}
			return d;
		}
	} // namespace multiview
}  // namespace fblib
