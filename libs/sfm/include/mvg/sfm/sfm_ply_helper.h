#ifndef MVG_SFM_SFM_PLY_HELPER_H
#define MVG_SFM_SFM_PLY_HELPER_H

#include <fstream>
#include <string>
#include <vector>

#include "mvg/math/numeric.h"
using namespace mvg::math;

namespace mvg{
	namespace sfm{

		/**
		 * \brief	导出3D点到ply文件
		 *
		 * \param	vec_points	3d点
		 * \param	file_name 	要导出的文件名
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		static bool exportToPly(const std::vector<Vec3> &vec_points,
			const std::string &file_name)
		{
			std::ofstream outfile;
			outfile.open(file_name.c_str(), std::ios_base::out);

			outfile << "ply"
				<< std::endl << "format ascii 1.0"
				<< std::endl << "element vertex " << vec_points.size()
				<< std::endl << "property float x"
				<< std::endl << "property float y"
				<< std::endl << "property float z"
				<< std::endl << "property uchar red"
				<< std::endl << "property uchar green"
				<< std::endl << "property uchar blue"
				<< std::endl << "end_header" << std::endl;

			for (size_t i = 0; i < vec_points.size(); ++i)
			{
				outfile << vec_points[i].transpose()
					<< " 255 255 255" << "\n";
			}
			bool is_ok = outfile.good();
			outfile.close();
			return is_ok;
		}

		/**
		 * \brief	导出3D点和相机位置到ply文件中
		 *
		 * \param	vec_points		  	3d点
		 * \param	vec_camera_pose   	相机位置
		 * \param	file_name		  	要导出的文件名
		 * \param	vec_colored_points	带颜色的3d点
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		static bool exportToPly(const std::vector<Vec3> & vec_points,
			const std::vector<Vec3> & vec_camera_pose,
			const std::string & file_name,
			const std::vector<Vec3> * vec_colored_points = NULL)
		{
			std::ofstream outfile;
			outfile.open(file_name.c_str(), std::ios_base::out);

			outfile << "ply"
				<< '\n' << "format ascii 1.0"
				<< '\n' << "element vertex " << vec_points.size() + vec_camera_pose.size()
				<< '\n' << "property float x"
				<< '\n' << "property float y"
				<< '\n' << "property float z"
				<< '\n' << "property uchar red"
				<< '\n' << "property uchar green"
				<< '\n' << "property uchar blue"
				<< '\n' << "end_header" << std::endl;

			for (size_t i = 0; i < vec_points.size(); ++i)  {
				if (vec_colored_points == NULL)
					outfile << vec_points[i].transpose()
					<< " 255 255 255" << "\n";
				else
					outfile << vec_points[i].transpose()
					<< " " << (*vec_colored_points)[i].transpose() << "\n";
			}

			for (size_t i = 0; i < vec_camera_pose.size(); ++i)  {
				outfile << vec_camera_pose[i].transpose()
					<< " 0 255 0" << "\n";
			}
			outfile.flush();
			bool is_ok = outfile.good();
			outfile.close();
			return is_ok;
		}

	} // namespace sfm
} // namespace mvg

#endif // MVG_SFM_SFM_PLY_HELPER_H

