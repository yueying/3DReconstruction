/*******************************************************************************
 * 文件： exif_simple.cpp
 * 时间： 2015/04/15 14:20
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： EXIF文件简单获取
 *
********************************************************************************/
#include <mvg/image/exif_simple.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

namespace mvg
{
	namespace image
	{
		EXIFSimple::EXIFSimple() : m_have_exif_info(false)
		{
		}

		EXIFSimple::EXIFSimple(const std::string & list_file_name) : m_have_exif_info(false)
		{
			open(list_file_name);
		}
		/**	析构函数
		*/
		EXIFSimple::~EXIFSimple()
		{}
		bool EXIFSimple::open(const std::string & list_file_name)
		{
			// 读取文件
			FILE *fp = fopen(list_file_name.c_str(), "rb");
			if (!fp) {
				return false;
			}
			fseek(fp, 0, SEEK_END);
			unsigned long fsize = ftell(fp);
			rewind(fp);
			std::vector<unsigned char> buf(fsize);
			if (fread(&buf[0], 1, fsize, fp) != fsize) {
				return false;
			}
			fclose(fp);

			// 解析EXIF
			int code = m_exif_info.parseFrom(&buf[0], fsize);
			if (code)
				m_have_exif_info = false;
			else
				m_have_exif_info = true;

			return m_have_exif_info;
		}

		size_t EXIFSimple::getWidth() const
		{
			return m_exif_info.m_image_width;
		}

		size_t EXIFSimple::getHeight() const
		{
			return m_exif_info.m_image_height;
		}

		float EXIFSimple::getFocalLength() const
		{
			return static_cast<float>(m_exif_info.m_focal_length);
		}

		std::string EXIFSimple::getBrand() const
		{
			std::string sbrand = m_exif_info.m_make;
			if (sbrand.empty()||sbrand=="")
			{
				return "";
			}
			//移除前后空格
			sbrand.erase(0, sbrand.find_first_not_of(' '));
			sbrand.erase(sbrand.find_last_not_of(' '));
			return sbrand;
		}

		std::string EXIFSimple::getModel() const
		{
			std::string smodel = m_exif_info.m_model;
			if (smodel.empty() || smodel == "")
			{
				return "";
			}
			//移除前后空格
			smodel.erase(0, smodel.find_first_not_of(' '));
			smodel.erase(smodel.find_last_not_of(' '));
			return smodel;
		}

		std::string EXIFSimple::getLensModel() const
		{
			return "";
		}

		bool EXIFSimple::doesHaveExifInfo() const
		{
			return m_have_exif_info;
		}

		std::string EXIFSimple::allExifData() const
		{
			std::ostringstream os;
			os
				<< "Camera make       : " << m_exif_info.m_make
				<< "Camera model      : " << m_exif_info.m_model
				<< "Software          : " << m_exif_info.m_software
				<< "Bits per sample   : " << m_exif_info.m_bits_per_sample
				<< "Image width       : " << m_exif_info.m_image_width
				<< "Image height      : " << m_exif_info.m_image_height
				<< "Image description : " << m_exif_info.m_image_description
				<< "Image orientation : " << m_exif_info.m_orientation
				<< "Image copyright   : " << m_exif_info.m_copyright
				<< "Image date/time   : " << m_exif_info.m_datetime
				<< "Original date/time: " << m_exif_info.m_datetime_original
				<< "Digitize date/time: " << m_exif_info.m_datetime_digitized
				<< "Subsecond time    : " << m_exif_info.m_sub_sec_time_original
				<< "Exposure time     : 1/time " << (unsigned)(1.0 / m_exif_info.m_exposure_time)
				<< "F-stop            : " << m_exif_info.m_f_number
				<< "ISO speed         : " << m_exif_info.m_iso_speed_ratings
				<< "Subject distance  : " << m_exif_info.m_subject_distance
				<< "Exposure bias     : EV" << m_exif_info.m_exposure_bias_value
				<< "Flash used?       : " << m_exif_info.m_flash
				<< "Metering mode     : " << m_exif_info.m_metering_mode
				<< "Lens focal length : mm\n" << m_exif_info.m_focal_length
				<< "35mm focal length : mm\n" << m_exif_info.m_focal_length_in_35mm
				<< "GPS Latitude      : deg ( deg, min, sec )\n" << "("
				<< m_exif_info.GeoLocation.latitude << ", "
				<< m_exif_info.GeoLocation.LatComponents.degrees << ", "
				<< m_exif_info.GeoLocation.LatComponents.minutes << ", "
				<< m_exif_info.GeoLocation.LatComponents.seconds << ", "
				<< m_exif_info.GeoLocation.LatComponents.direction << ")"
				<< "GPS Longitude     : deg ( deg, min, sec )\n" << "("
				<< m_exif_info.GeoLocation.longitude << ", "
				<< m_exif_info.GeoLocation.LonComponents.degrees << ", "
				<< m_exif_info.GeoLocation.LonComponents.minutes << ", "
				<< m_exif_info.GeoLocation.LonComponents.seconds << ", "
				<< m_exif_info.GeoLocation.LonComponents.direction << ")"
				<< "GPS Altitude      : m" << m_exif_info.GeoLocation.altitude;
			return os.str();
		}
	}
}