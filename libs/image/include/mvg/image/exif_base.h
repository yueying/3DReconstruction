/*******************************************************************************
 * 文件： exif_base.h
 * 时间： 2015/04/15 18:45
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 定义EXIF解析获取基类
 *
********************************************************************************/
#ifndef MVG_IMAGE_EXIF_BASE_H_
#define MVG_IMAGE_EXIF_BASE_H_
#include <mvg/image/link_pragmas.h>

#include <string>

namespace mvg
{
	namespace image
	{
		class IMAGE_IMPEXP EXIFBase
		{
		public:

			virtual ~EXIFBase(){};
			/**	打开文件进行检查和解析
			*/
			virtual bool open(const std::string & list_file_name) = 0;

			/**验证文件是否有EXIF头信息
			*/
			virtual bool doesHaveExifInfo() const = 0;
			/**	得到图像宽度
			 */
			virtual size_t getWidth() const = 0;
			/**	得到图像高度
			 */
			virtual size_t getHeight() const = 0;
			/**	得到焦距信息
			 */
			virtual float getFocalLength() const = 0;
			/**	得到相机品牌信息
			 */
			virtual std::string getBrand() const = 0;
			/**	得到相机型号
			 */
			virtual std::string getModel() const = 0;
			/**	得到镜头型号
			 */
			virtual std::string getLensModel() const = 0;

			/**	打印出所有数据
			 */
			virtual std::string allExifData() const = 0;

		};
	}
}


#endif // MVG_IMAGE_EXIF_BASE_H_