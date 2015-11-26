#ifndef FBLIB_SFM_IO_HELPER_H_
#define FBLIB_SFM_IO_HELPER_H_
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "fblib/math/numeric.h"
#include "fblib/utils/string_utils.h"

using namespace fblib::math;
using namespace fblib::utils;

namespace fblib{
	namespace feature{
		/**	给出相机图片及ID
		 */
		struct CameraInfo
		{
			std::string image_name;//!<对应相机图片
			size_t intrinsic_id;//!<图片id
		};
		/**相机内参信息*/
		struct IntrinsicCameraInfo
		{
			size_t width, height;//!<图像的宽度和高度
			float focal;//!< 焦距值
			Mat3 camera_matrix;//!<相机内参矩阵
			bool is_known_intrinsic; //!< 当解析内参值为6和12时为true
			std::string camera_maker;//!<相机厂商
			std::string camera_model;//!<相机型号
			/**用于判断两个相机内参是否相等*/
			bool operator() (const IntrinsicCameraInfo  &ci1, const IntrinsicCameraInfo &ci2)const
			{
				bool is_equal = false;
				if (ci1.camera_maker.compare("") != 0 && ci1.camera_model.compare("") != 0)
				{
					if (ci1.camera_maker.compare(ci2.camera_maker) == 0
						&& ci1.camera_model.compare(ci2.camera_model) == 0
						&& ci1.width == ci2.width
						&& ci1.height == ci2.height
						&& ci1.focal == ci2.focal)
					{
						is_equal = true;
					}
					else
					{
						if (is_known_intrinsic)
							is_equal = (ci1.camera_matrix == ci2.camera_matrix);
					}
				}
				return !is_equal;
			}
		};

		/**
		 * \brief	导入图像文件列表信息，获得相机内部信息，主要有如下几种 1、相机没有exif数据 2、相机包含exif数据，可以在数据库中找到
		 * 			3、相机包含exif数据，在数据库中没有找到 4、相机已知内参信息.
		 *
		 * \param	list_file_name			  	包含图像文件列表信息的list文件.
		 * \param [in,out]	vec_camera_info	一系列图像包含对应id.
		 * \param [in,out]	vec_cameras_intrinsic	图像对应的内参信息.
		 * \param	is_verbose				  	true表示exif信息详细.
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		static bool loadImageList(const std::string list_file_name,
			std::vector<CameraInfo> &vec_camera_info,
			std::vector<IntrinsicCameraInfo> &vec_cameras_intrinsic,
			bool is_verbose = true)
		{
			typedef std::set<IntrinsicCameraInfo, IntrinsicCameraInfo> SetIntrinsicCameraInfo;
			SetIntrinsicCameraInfo focal_group_set;

			std::ifstream in(list_file_name.c_str());
			if (!in.is_open())  {
				std::cerr << std::endl
					<< "Impossible to read the specified file." << std::endl;
			}
			std::string str_line;
			std::vector<std::string> vec_str;
			while (getline(in, str_line))
			{
				vec_str.clear();
				IntrinsicCameraInfo intrinsic_camera_info;
				fblib::utils::split(str_line, ";", vec_str);
				if (vec_str.size() == 1)
				{
					std::cerr << "Invalid input file" << std::endl;
					in.close();
					return false;
				}
				std::stringstream oss;
				oss.clear(); oss.str(vec_str[1]);
				size_t width, height;
				oss >> width;
				oss.clear(); oss.str(vec_str[2]);
				oss >> height;

				intrinsic_camera_info.width = width;
				intrinsic_camera_info.height = height;

				switch (vec_str.size())
				{
				case 3: // 相机没有exif数据
				{
					intrinsic_camera_info.focal = -1;
					intrinsic_camera_info.is_known_intrinsic = false;
					intrinsic_camera_info.camera_maker = "";
					intrinsic_camera_info.camera_model = "";
				}
					break;
				case 5: // 相机包含exif数据，可以在数据库中没有找到
				{
					intrinsic_camera_info.focal = -1;
					intrinsic_camera_info.is_known_intrinsic = false;
					intrinsic_camera_info.camera_maker = vec_str[3];
					intrinsic_camera_info.camera_model = vec_str[4];
				}
					break;
				case  6: // 相机包含exif数据，在数据库中找到相关信息
				{
					oss.clear(); oss.str(vec_str[3]);
					float focal;
					oss >> focal;
					intrinsic_camera_info.focal = focal;
					intrinsic_camera_info.is_known_intrinsic = true;
					intrinsic_camera_info.camera_maker = vec_str[4];
					intrinsic_camera_info.camera_model = vec_str[5];

					Mat3 K;
					K << focal, 0, float(width) / 2.f,
						0, focal, float(height) / 2.f,
						0, 0, 1;
					intrinsic_camera_info.camera_matrix = K;

				}
					break;
				case 12: // 直接获得已知的内参信息
				{
					intrinsic_camera_info.is_known_intrinsic = true;
					intrinsic_camera_info.camera_maker = intrinsic_camera_info.camera_model = "";

					Mat3 K = Mat3::Identity();

					oss.clear(); oss.str(vec_str[3]);
					oss >> K(0, 0);
					oss.clear(); oss.str(vec_str[4]);
					oss >> K(0, 1);
					oss.clear(); oss.str(vec_str[5]);
					oss >> K(0, 2);
					oss.clear(); oss.str(vec_str[6]);
					oss >> K(1, 0);
					oss.clear(); oss.str(vec_str[7]);
					oss >> K(1, 1);
					oss.clear(); oss.str(vec_str[8]);
					oss >> K(1, 2);
					oss.clear(); oss.str(vec_str[9]);
					oss >> K(2, 0);
					oss.clear(); oss.str(vec_str[10]);
					oss >> K(2, 1);
					oss.clear(); oss.str(vec_str[11]);
					oss >> K(2, 2);

					intrinsic_camera_info.camera_matrix = K;
					// 直接使用内参矩阵第一个参数作为焦距值，没有根据图像大小进行修改;
					intrinsic_camera_info.focal = static_cast<float>(K(0, 0));
				}
					break;
				default:
				{
					std::cerr << "Invalid line : wrong number of arguments" << std::endl;
				}
				}

				std::pair<SetIntrinsicCameraInfo::iterator, bool> ret = focal_group_set.insert(intrinsic_camera_info);
				if (ret.second)
				{
					vec_cameras_intrinsic.push_back(intrinsic_camera_info);
				}
				size_t id = std::distance(ret.first, focal_group_set.end()) - 1;
				CameraInfo camera_info;
				camera_info.image_name = vec_str[0];
				camera_info.intrinsic_id = id;
				vec_camera_info.push_back(camera_info);

				vec_str.clear();
			}
			in.close();
			return !(vec_camera_info.empty());
		}

		/**
		 * \brief	导入图像list文件.
		 *
		 * \param	list_file_name			  	输入存储图像列表的list文件名
		 * \param [in,out]	vec_camera_image_name	图像名称列表
		 * \param	is_verbose				  	true表示exif信息详细.
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		static bool loadImageList(const std::string list_file_name,
			std::vector<std::string> & vec_camera_image_name,
			bool is_verbose = true)
		{
			vec_camera_image_name.clear();
			std::vector<fblib::feature::CameraInfo> vec_camera_info;
			std::vector<fblib::feature::IntrinsicCameraInfo> vec_cameras_intrinsic;
			if (loadImageList(list_file_name,
				vec_camera_info,
				vec_cameras_intrinsic,
				is_verbose))
			{
				for (std::vector<fblib::feature::CameraInfo>::const_iterator
					iter_camera_info = vec_camera_info.begin();
					iter_camera_info != vec_camera_info.end();
				iter_camera_info++)
				{
					vec_camera_image_name.push_back(iter_camera_info->image_name);
				}
			}
			return (!vec_camera_image_name.empty());
		}

	} // namespace sfmio
} // namespace fblib

#endif // FBLIB_SFM_IO_HELPER_H_

