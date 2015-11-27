/*******************************************************************************
 * 文件： exif_info.cpp
 * 时间： 2015/04/13 18:51
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： JPEG头文件的解析 IFD：Image File Directory
 *
********************************************************************************/
#include <mvg/image/exif_info.h>

namespace mvg
{
	namespace image
	{
		// IF Entry 
		struct IFEntry {
			// 原始字段
			unsigned short tag;
			unsigned short format;
			unsigned data;
			unsigned length;

			// 解析字段
			std::string val_string;
			unsigned short val_16;
			unsigned val_32;
			double val_rational;
			unsigned char val_byte;
		};

		// 帮助函数
		unsigned int parse32(const unsigned char *buf, bool intel) {
			if (intel)
				return ((unsigned)buf[3] << 24) |
				((unsigned)buf[2] << 16) |
				((unsigned)buf[1] << 8) |
				buf[0];

			return ((unsigned)buf[0] << 24) |
				((unsigned)buf[1] << 16) |
				((unsigned)buf[2] << 8) |
				buf[3];
		}

		unsigned short parse16(const unsigned char *buf, bool intel) {
			if (intel)
				return ((unsigned)buf[1] << 8) | buf[0];
			return ((unsigned)buf[0] << 8) | buf[1];
		}

		std::string parseEXIFString(const unsigned char *buf,
			const unsigned num_components,
			const unsigned data,
			const unsigned base,
			const unsigned len) {
			std::string value;
			if (num_components <= 4)
				value.assign((const char*)&data, num_components);
			else {
				if (base + data + num_components <= len)
					value.assign((const char*)(buf + base + data), num_components);
			}
			return value;
		}

		double parseEXIFRational(const unsigned char *buf, bool intel) {
			double numerator = 0;
			double denominator = 1;

			numerator = (double)parse32(buf, intel);
			denominator = (double)parse32(buf + 4, intel);
			if (denominator < 1e-20)
				return 0;
			return numerator / denominator;
		}

		IFEntry parseIFEntry(const unsigned char *buf,
			const unsigned offs,
			const bool alignIntel,
			const unsigned base,
			const unsigned len) {
			IFEntry result;

			// 每个条目的组成
			// 2 bytes: 标记数字(数据域)
			// 2 bytes: 数据格式
			// 4 bytes: 组件数
			// 4 bytes: 数据值或数据值的偏移量
			result.tag = parse16(buf + offs, alignIntel);
			result.format = parse16(buf + offs + 2, alignIntel);
			result.length = parse32(buf + offs + 4, alignIntel);
			result.data = parse32(buf + offs + 8, alignIntel);

			// 按照指定格式进行解析
			switch (result.format) {
			case 1:
				result.val_byte = (unsigned char)*(buf + offs + 8);
				break;
			case 2:
				result.val_string = parseEXIFString(buf, result.length, result.data, base, len);
				break;
			case 3:
				result.val_16 = parse16((const unsigned char *)buf + offs + 8, alignIntel);
				break;
			case 4:
				result.val_32 = result.data;
				break;
			case 5:
				if (base + result.data + 8 <= len)
					result.val_rational = parseEXIFRational(buf + base + result.data, alignIntel);
				break;
			case 7:
			case 9:
			case 10:
				break;
			default:
				result.tag = 0xFF;
			}
			return result;
		}

		int EXIFInfo::parseFrom(const unsigned char *buf, unsigned len) {
			// 安全性检测:所有的 JPEG 文件开始于0xFFD8，结束于0xFFD9
			// 这种检查还可以确保用户提供了正确值的长度
			if (!buf || len < 4)
				return PARSE_EXIF_ERROR_NO_EXIF;
			if (buf[0] != 0xFF || buf[1] != 0xD8)
				return PARSE_EXIF_ERROR_NO_JPEG;
			if (buf[len - 2] != 0xFF || buf[len - 1] != 0xD9)
				return PARSE_EXIF_ERROR_NO_JPEG;
			clear();

			// 扫描EXIF头(0xFF 0xE1)，做安全性检查寻找字节"Exif\0\0".该标识的长度数据是在Motorola
			// 字节顺序，在parse16()提供参数 'false'
			// 标识至少包含TIFF头信息,否则EXIF数据已损坏
			// 所以此处最小数据长度必须是
			//   2 bytes: 段长度
			//   6 bytes: "Exif\0\0" 字符串
			//   2 bytes: TIFF 头 (为 "II" 或 "MM" )
			//   2 bytes: TIFF 改变 (两字节 0x2a00 在 Motorola 字节顺序下)
			//   4 bytes: 第一个IFD的偏移量
			// =========
			//  16 bytes
			unsigned offs = 0;        // 在缓冲区中当前偏移量
			for (offs = 0; offs < len - 1; offs++)
				if (buf[offs] == 0xFF && buf[offs + 1] == 0xE1)
					break;
			if (offs + 4 > len)
				return PARSE_EXIF_ERROR_NO_EXIF;
			offs += 2;
			unsigned short section_length = parse16(buf + offs, false);
			if (offs + section_length > len || section_length < 16)
				return PARSE_EXIF_ERROR_CORRUPT;
			offs += 2;

			return parseFromEXIFSegment(buf + offs, len - offs);
		}

		int EXIFInfo::parseFrom(const std::string &data) {
			return parseFrom((const unsigned char *)data.data(), data.length());
		}

		
		//对EXIF段进行解析 'buf'必须开始于EXIF TIFF，为字节"Exif\0\0" ；'len'为buffer的长度
		int EXIFInfo::parseFromEXIFSegment(const unsigned char *buf, unsigned len) {
			bool alignIntel = true;     // 字节对齐方式 (定义在 EXIF 头信息中)
			unsigned offs = 0;        // 缓冲区中当前偏移量
			if (!buf || len < 6)
				return PARSE_EXIF_ERROR_NO_EXIF;

			if (!std::equal(buf, buf + 6, "Exif\0\0"))
				return PARSE_EXIF_ERROR_NO_EXIF;
			offs += 6;

			// 现在解析 TIFF 头. 前两个字节为"II"或者"MM"表示Intel或者 Motorola字节对齐方式
			// 安全性检查，接下来的两个字节必须是0x2a. 最后4个字节是第一个IFD的偏移量 
			// 将会被添加到全局偏移计数器中
			// 这块的最小大小8个字节:
			//  2 bytes: 'II' or 'MM'
			//  2 bytes: 0x002a
			//  4 bytes: 第一个IDF的偏移量
			// -----------------------------
			//  8 bytes
			if (offs + 8 > len)
				return PARSE_EXIF_ERROR_CORRUPT;
			unsigned tiff_header_start = offs;
			if (buf[offs] == 'I' && buf[offs + 1] == 'I')
				alignIntel = true;
			else {
				if (buf[offs] == 'M' && buf[offs + 1] == 'M')
					alignIntel = false;
				else
					return PARSE_EXIF_ERROR_UNKNOWN_BYTEALIGN;
			}
			this->m_byte_align = alignIntel;
			offs += 2;
			if (0x2a != parse16(buf + offs, alignIntel))
				return PARSE_EXIF_ERROR_CORRUPT;
			offs += 2;
			unsigned first_ifd_offset = parse32(buf + offs, alignIntel);
			offs += first_ifd_offset - 4;
			if (offs >= len)
				return PARSE_EXIF_ERROR_CORRUPT;

			// 先解析第一个图像文件目录 (IFD0，主图像).
			// 一个IFD包含多个12字节的目录条目
			// IFD段的前2个字节包含目录条数
			// 最后4个字节包含一个偏移量对下一个IFD
			// 这表明IFD段必须包含6 + 12 * num个字节数
			if (offs + 2 > len)
				return PARSE_EXIF_ERROR_CORRUPT;
			int num_entries = parse16(buf + offs, alignIntel);
			if (offs + 6 + 12 * num_entries > len)
				return PARSE_EXIF_ERROR_CORRUPT;
			offs += 2;
			unsigned exif_sub_ifd_offset = len;
			unsigned gps_sub_ifd_offset = len;
			while (--num_entries >= 0) {
				IFEntry result = parseIFEntry(buf, offs, alignIntel, tiff_header_start, len);
				offs += 12;
				switch (result.tag) {
				case 0x102:
					// 取样数
					if (result.format == 3)
						this->m_bits_per_sample = result.val_16;
					break;

				case 0x10E:
					// 图像描述
					if (result.format == 2)
						this->m_image_description = result.val_string;
					break;

				case 0x10F:
					// 相机制造商
					if (result.format == 2)
						this->m_make = result.val_string;
					break;

				case 0x110:
					// 相机型号
					if (result.format == 2)
						this->m_model = result.val_string;
					break;

				case 0x112:
					// 图片方向
					if (result.format == 3)
						this->m_orientation = result.val_16;
					break;

				case 0x131:
					// 使用软件
					if (result.format == 2)
						this->m_software = result.val_string;
					break;

				case 0x132:
					// EXIF/TIFF 文件修改的日期与时间
					if (result.format == 2)
						this->m_datetime = result.val_string;
					break;

				case 0x8298:
					// 文件的版权信息
					if (result.format == 2)
						this->m_copyright = result.val_string;
					break;

				case 0x8825:
					// GPS IFS 偏移量
					gps_sub_ifd_offset = tiff_header_start + result.data;
					break;

				case 0x8769:
					// EXIF SubIFD 偏移量
					exif_sub_ifd_offset = tiff_header_start + result.data;
					break;
				}
			}

			// 如果存在，则跳转到EXIF SubIFD，解析其所有信息。
			// 这边要注意的是EXIF SubIFD可能不存在
			// EXIF SubIFD包含了很多有用的信息
			if (exif_sub_ifd_offset + 4 <= len) {
				offs = exif_sub_ifd_offset;
				int num_entries = parse16(buf + offs, alignIntel);
				if (offs + 6 + 12 * num_entries > len)
					return PARSE_EXIF_ERROR_CORRUPT;
				offs += 2;
				while (--num_entries >= 0) {
					IFEntry result = parseIFEntry(buf, offs, alignIntel, tiff_header_start, len);
					switch (result.tag) {
					case 0x829a:
						// 曝光时间，单位秒
						if (result.format == 5)
							this->m_exposure_time = result.val_rational;
						break;

					case 0x829d:
						// 相机光圈数
						if (result.format == 5)
							this->m_f_number = result.val_rational;
						break;

					case 0x8827:
						// ISO 感光度
						if (result.format == 3)
							this->m_iso_speed_ratings = result.val_16;
						break;

					case 0x9003:
						//原始文件的日期和时间
						if (result.format == 2)
							this->m_datetime_original = result.val_string;
						break;

					case 0x9004:
						// 数字化的日期和时间
						if (result.format == 2)
							this->m_datetime_digitized = result.val_string;
						break;

					case 0x9201:
						// 快门速度(曝光时间的倒数)
						if (result.format == 5)
							this->m_shutter_speed_value = result.val_rational;
						break;

					case 0x9204:
						// 曝光补偿值
						if (result.format == 5)
							this->m_exposure_bias_value = result.val_rational;
						break;

					case 0x9206:
						// 对焦点的距离
						if (result.format == 5)
							this->m_subject_distance = result.val_rational;
						break;

					case 0x9209:
						// 是否使用闪光
						if (result.format == 3)
							this->m_flash = result.data ? 1 : 0;
						break;

					case 0x920a:
						// 焦距
						if (result.format == 5)
							this->m_focal_length = result.val_rational;
						break;

					case 0x9207:
						// 测光模式
						if (result.format == 3)
							this->m_metering_mode = result.val_16;
						break;

					case 0x9291:
						// 图像被拍摄的低于秒级的时间
						if (result.format == 2)
							this->m_sub_sec_time_original = result.val_string;
						break;

					case 0xa002:
						// EXIF 图像宽度
						if (result.format == 4)
							this->m_image_width = result.val_32;
						else
							if (result.format == 3)
								this->m_image_width = result.val_16;
							else
								this->m_image_width = result.data;
						break;

					case 0xa003:
						// EXIF 图像高度
						if (result.format == 4)
							this->m_image_height = result.val_32;
						else
							if (result.format == 3)
								this->m_image_height = result.val_16;
							else
								this->m_image_height = result.data;
						break;

					case 0xa405:
						// 35mm等效焦距
						if (result.format == 3)
							this->m_focal_length_in_35mm = result.val_16;
						break;
					}
					offs += 12;
				}
			}

			// 如果GPS存在则跳转到 GPS SubIFD 进行解析
			if (gps_sub_ifd_offset + 4 <= len) {
				offs = gps_sub_ifd_offset;
				int num_entries = parse16(buf + offs, alignIntel);
				if (offs + 6 + 12 * num_entries > len)
					return PARSE_EXIF_ERROR_CORRUPT;
				offs += 2;
				while (--num_entries >= 0) {
					unsigned short tag = parse16(buf + offs, alignIntel);
					unsigned short format = parse16(buf + offs + 2, alignIntel);
					unsigned length = parse32(buf + offs + 4, alignIntel);
					unsigned data = parse32(buf + offs + 8, alignIntel);
					switch (tag) {
					case 1:
						// GPS 南 或者 北
						this->GeoLocation.LatComponents.direction = *(buf + offs + 8);
						if ('S' == this->GeoLocation.LatComponents.direction)
							this->GeoLocation.latitude = -this->GeoLocation.latitude;
						break;

					case 2:
						// GPS 纬度信息
						if (format == 5 && length == 3) {
							this->GeoLocation.LatComponents.degrees =
								parseEXIFRational(buf + data + tiff_header_start, alignIntel);
							this->GeoLocation.LatComponents.minutes =
								parseEXIFRational(buf + data + tiff_header_start + 8, alignIntel);
							this->GeoLocation.LatComponents.seconds =
								parseEXIFRational(buf + data + tiff_header_start + 16, alignIntel);
							this->GeoLocation.latitude =
								this->GeoLocation.LatComponents.degrees +
								this->GeoLocation.LatComponents.minutes / 60 +
								this->GeoLocation.LatComponents.seconds / 3600;
							if ('S' == this->GeoLocation.LatComponents.direction)
								this->GeoLocation.latitude = -this->GeoLocation.latitude;
						}
						break;

					case 3:
						// GPS 东或西
						this->GeoLocation.LonComponents.direction = *(buf + offs + 8);
						if ('W' == this->GeoLocation.LonComponents.direction)
							this->GeoLocation.longitude = -this->GeoLocation.longitude;
						break;

					case 4:
						// GPS 经度信息
						if (format == 5 && length == 3) {
							this->GeoLocation.LonComponents.degrees =
								parseEXIFRational(buf + data + tiff_header_start, alignIntel);
							this->GeoLocation.LonComponents.minutes =
								parseEXIFRational(buf + data + tiff_header_start + 8, alignIntel);
							this->GeoLocation.LonComponents.seconds =
								parseEXIFRational(buf + data + tiff_header_start + 16, alignIntel);
							this->GeoLocation.longitude =
								this->GeoLocation.LonComponents.degrees +
								this->GeoLocation.LonComponents.minutes / 60 +
								this->GeoLocation.LonComponents.seconds / 3600;
							if ('W' == this->GeoLocation.LonComponents.direction)
								this->GeoLocation.longitude = -this->GeoLocation.longitude;
						}
						break;

					case 5:
						// 相对于海平面的高度
						this->GeoLocation.altitude_ref = *(buf + offs + 8);
						if (1 == this->GeoLocation.altitude_ref)
							this->GeoLocation.altitude = -this->GeoLocation.altitude;
						break;

					case 6:
						// 与海平面的相对位置
						if (format == 5) {
							this->GeoLocation.altitude =
								parseEXIFRational(buf + data + tiff_header_start, alignIntel);
							if (1 == this->GeoLocation.altitude_ref)
								this->GeoLocation.altitude = -this->GeoLocation.altitude;
						}
						break;
					}
					offs += 12;
				}
			}

			return PARSE_EXIF_SUCCESS;
		}

		//给出默认初始值
		void EXIFInfo::clear() {
			
			m_image_description = "";
			m_make = "";
			m_model = "";
			m_software = "";
			m_datetime = "";
			m_datetime_original = "";
			m_datetime_digitized = "";
			m_sub_sec_time_original = "";
			m_copyright = "";

			m_byte_align = 0;
			m_orientation = 0;

			m_bits_per_sample = 0;
			m_exposure_time = 0;
			m_f_number = 0;
			m_iso_speed_ratings = 0;
			m_shutter_speed_value = 0;
			m_exposure_bias_value = 0;
			m_subject_distance = 0;
			m_focal_length = 0;
			m_focal_length_in_35mm = 0;
			m_flash = 0;
			m_metering_mode = 0;
			m_image_width = 0;
			m_image_height = 0;

			// Geolocation
			GeoLocation.latitude = 0;
			GeoLocation.longitude = 0;
			GeoLocation.altitude = 0;
			GeoLocation.altitude_ref = 0;
			GeoLocation.LatComponents.degrees = 0;
			GeoLocation.LatComponents.minutes = 0;
			GeoLocation.LatComponents.seconds = 0;
			GeoLocation.LatComponents.direction = 0;
			GeoLocation.LonComponents.degrees = 0;
			GeoLocation.LonComponents.minutes = 0;
			GeoLocation.LonComponents.seconds = 0;
			GeoLocation.LonComponents.direction = 0;
		}

	}
}