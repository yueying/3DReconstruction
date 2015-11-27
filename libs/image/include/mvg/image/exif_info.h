/*******************************************************************************
 * 文件： exif_info.h
 * 时间： 2015/04/13 18:34
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 对jpg图像头进行解析
 *
 ********************************************************************************/
#ifndef MVG_IMAGE_EXIF_INFO_H_
#define MVG_IMAGE_EXIF_INFO_H_

#include <mvg/image/link_pragmas.h>
#include <string>

namespace mvg
{
	namespace image
	{
		//!<解析成功
		#define PARSE_EXIF_SUCCESS                    0
		//!<无效的JPEG文件
		#define PARSE_EXIF_ERROR_NO_JPEG              1982
		//!<JPEG文件中没有EXIF头信息
		#define PARSE_EXIF_ERROR_NO_EXIF              1983
		//!<在EXIF文件中指定字节对齐方式未知
		#define PARSE_EXIF_ERROR_UNKNOWN_BYTEALIGN    1984
		//!<EXIF头文件找到，但是数据不可靠
		#define PARSE_EXIF_ERROR_CORRUPT              1985

		/**	用于存储和解析jpg图像中的exif头信息
		 */
		class IMAGE_IMPEXP EXIFInfo {
		public:

			EXIFInfo() {
				clear();
			}
			/**	对整个JPEG图像缓冲区的解析函数
			 * \param data 指向JPEG图像的指针
			 * \param length JPEG图像的长度
			 */
			int parseFrom(const unsigned char *data, unsigned length);
			int parseFrom(const std::string &data);

			/**	EXIF段的解析函数，供parseFrom()内部使用，设为public可以供特殊情况只有EXIF的时候使用
			 * 例如以"Exif\0\0"开头
			 */
			int parseFromEXIFSegment(const unsigned char *buf, unsigned len);

			/**	将所有成员值都设为默认值
			 */
			void clear();

		public:
			// parseFrom()解析字符串
			char m_byte_align;                   //!< 0 = Motorola 字节对齐, 1 = Intel 
			std::string m_image_description;     //!< 图像描述
			std::string m_make;                 //!< 相机制造商的名称
			std::string m_model;                //!< 相机型号

			// 0: unspecified in EXIF data
			// 1: upper left of image
			// 3: lower right of image
			// 6: upper right of image
			// 8: lower left of image
			// 9: undefined
			unsigned short m_orientation;       //!< 图像位置
			
			unsigned short m_bits_per_sample;     //!< 每个组件的位数
			std::string m_software;             //!< 使用软件
			std::string m_datetime;             //!< 文件改变的日期与时间
			std::string m_datetime_original;     //!< 原始文件的日期和时间(可能不存在)
			std::string m_datetime_digitized;    //!< 数字化的日期和时间(可能不存在)
			std::string m_sub_sec_time_original;   //!< 图像被拍摄的低于秒级的时间
			std::string m_copyright;            //!< 文件的版权信息
			double m_exposure_time;              //!< 曝光时间，单位秒
			double m_f_number;                   //!< 相机光圈数
			unsigned short m_iso_speed_ratings;   //!< ISO 感光度
			double m_shutter_speed_value;         //!< 快门速度(曝光时间的倒数)
			double m_exposure_bias_value;         //!< EV中的曝光补偿值
			double m_subject_distance;           //!< 对焦点的距离 单位米
			double m_focal_length;               //!< 镜头的焦距 单位毫米
			unsigned short m_focal_length_in_35mm; //!< 35mm等效焦距
			char m_flash;                       //!< 0 = 无闪光, 1 = 使用闪光

			// 1: average
			// 2: center weighted average
			// 3: spot
			// 4: multi-spot
			// 5: multi-segment
			unsigned short m_metering_mode;      //!< 测光模式
			
			unsigned m_image_width;              //!< 在 EXIF 数据中记录的图像宽度
			unsigned m_image_height;             //!< 在 EXIF 数据中记录的图像高度
			struct Geolocation_t {            //!< 文件中嵌入的GPS信息
				double latitude;                  // 十进制表示 图像的纬度
				double longitude;                 // 十进制表示 图像的经度
				double altitude;                  // 以米为单位 相对于海平面的高度
				char altitude_ref;                 // 与海平面的相对位置0 = 海平面之上, -1 = 海平面之下
				struct Coord_t {
					double degrees;
					double minutes;
					double seconds;
					char direction;
				} LatComponents, LonComponents;   // 经度和纬度解析成 度/分/秒 
			} GeoLocation;
			
		};
	}
}


#endif // MVG_IMAGE_EXIF_INFO_H_