/*******************************************************************************
 * 文件： exif_simple.h
 * 时间： 2015/04/15 19:59
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 对EXIF头文件的简单解析获取
 *
********************************************************************************/
#ifndef MVG_IMAGE_EXIF_SIMPLE_H_
#define MVG_IMAGE_EXIF_SIMPLE_H_
#include <mvg/image/link_pragmas.h> 
#include <mvg/image/exif_base.h>
#include <mvg/image/exif_info.h>

namespace mvg
{
	namespace image
	{
		/**	对EXIF简单获取
		 */
		class IMAGE_IMPEXP EXIFSimple : public EXIFBase
		{
		public:
			/**	默认构造函数
			 */
			EXIFSimple();
			/**	打开指定文件
			 */
			EXIFSimple(const std::string &list_file_name);
			/**	析构函数
			 */
			~EXIFSimple();
			/**	打开文件进行检查和解析
			*/
			bool open(const std::string & list_file_name);

			/**验证文件是否有EXIF头信息
			*/
			bool doesHaveExifInfo() const;
			/**	得到图像宽度
			*/
			size_t getWidth() const;
			/**	得到图像高度
			*/
			size_t getHeight() const;
			/**	得到焦距信息
			*/
			float getFocalLength() const;
			/**	得到相机品牌信息
			*/
			std::string getBrand() const;
			/**	得到相机型号
			*/
			std::string getModel() const;
			/**	得到镜头型号
			*/
			std::string getLensModel() const;

			/**	打印出所有数据
			*/
			std::string allExifData() const;
		private:
			EXIFInfo m_exif_info;
			bool m_have_exif_info;

		};
	}
}

#endif // MVG_IMAGE_EXIF_SIMPLE_H_