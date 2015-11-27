#ifndef MVG_IMAGE_IMAGE_IO_H_
#define MVG_IMAGE_IMAGE_IO_H_

#include "mvg/image/link_pragmas.h"
#include "mvg/image/image_container.h"
#include "mvg/image/pixel_types.h"
#include "mvg/image/image_converter.h"

namespace mvg {
	namespace image{

		enum Format {
			Pnm, Png, Jpg, Unknown
		};

		/**
		 * \brief	根据给定的图像文件，得出图像文件类型
		 *
		 * \param	c	输入图像文件地址
		 *
		 * \return	图像文件类型
		 */
		Format GetFormat(const char *c);

		/**
		 * \brief	根据给定的文件路径读取图片
		 *
		 * \tparam	T	图片类型
		 * \param	parameter1		  	图片文件路径
		 * \param [in,out]	parameter2	返回图片
		 *
		 * \return	读取的图片
		 */
		template<typename T>
		int readImage(const char *, Image<T> *);

		/// Open an png image with unsigned char as memory target.
		/// The memory point must be null as input.
		int IMAGE_IMPEXP readImage(const char *, std::vector<unsigned char> *, int * w, int * h, int * depth);
		int ReadPng(const char *, std::vector<unsigned char> *, int * w, int * h, int * depth);
		int ReadPngStream(FILE *, std::vector<unsigned char> *, int * w, int * h, int * depth);

		template<typename T>
		int writeImage(const char *, const Image<T>&);

		int IMAGE_IMPEXP writeImage(const char *, const std::vector<unsigned char>& array, int w, int h, int depth);
		int WritePng(const char *, const std::vector<unsigned char>& array, int w, int h, int depth);
		int WritePngStream(FILE *, const std::vector<unsigned char>& array, int w, int h, int depth);

		template<typename T>
		int WriteJpg(const char *, const Image<T>&, int quality = 90);
		int WriteJpg(const char *, const std::vector<unsigned char>& array, int w, int h, int depth, int quality = 90);
		int WriteJpgStream(FILE *, const std::vector<unsigned char>& array, int w, int h, int depth, int quality = 90);


		/// Open a jpg image with unsigned char as memory target.
		/// The memory point must be null as input.
		int ReadJpg(const char *, std::vector<unsigned char> *, int * w, int * h, int * depth);
		int ReadJpgStream(FILE *, std::vector<unsigned char> *, int * w, int * h, int * depth);

		int ReadPnm(const char *, std::vector<unsigned char> *, int * w, int * h, int * depth);
		int ReadPnmStream(FILE *, std::vector<unsigned char> *, int * w, int * h, int * depth);

		int WritePnm(const char *, const std::vector<unsigned char>& array, int w, int h, int depth);
		int WritePnmStream(FILE *, const std::vector<unsigned char>& array, int w, int h, int depth);

		template<>
		inline int readImage(const char * path, Image<unsigned char> * im)
		{
			std::vector<unsigned char> ptr;
			int w, h, depth;
			int res = readImage(path, &ptr, &w, &h, &depth);
			if (res == 1 && depth == 1) {
				//convert raw array to Image
				(*im) = Eigen::Map<Image<unsigned char>::Base>(&ptr[0], h, w);
			}
			else
				if (res == 1 && depth == 3) {
				//-- Must convert RGB to gray
				RGBColor * ptrCol = (RGBColor*)&ptr[0];
				Image<RGBColor> rgbColIm;
				rgbColIm = Eigen::Map<Image<RGBColor>::Base>(ptrCol, h, w);
				//convert RGB to gray
				ConvertPixelType(rgbColIm, im);
				}
				else
					if (res == 1 && depth == 4) {
				//-- Must convert RGBA to gray
				RGBAColor * ptrCol = (RGBAColor*)&ptr[0];
				Image<RGBAColor> rgbaColIm;
				rgbaColIm = Eigen::Map<Image<RGBAColor>::Base>(ptrCol, h, w);
				//convert RGBA to gray
				ConvertPixelType(rgbaColIm, im);
					}
					else if (depth != 1)
						return 0;
			return res;
		}

		template<>
		inline int readImage(const char * path, Image<RGBColor> * im)
		{
			std::vector<unsigned char> ptr;
			int w, h, depth;
			int res = readImage(path, &ptr, &w, &h, &depth);
			if (res == 1 && depth == 3) {
				RGBColor * ptrCol = (RGBColor*)&ptr[0];
				//convert raw array to Image
				(*im) = Eigen::Map<Image<RGBColor>::Base>(ptrCol, h, w);
			}
			else
				if (res == 1 && depth == 4) {
				//-- Must convert RGBA to RGB
				RGBAColor * ptrCol = (RGBAColor*)&ptr[0];
				Image<RGBAColor> rgbaColIm;
				rgbaColIm = Eigen::Map<Image<RGBAColor>::Base>(ptrCol, h, w);
				//convert RGBA to RGB
				ConvertPixelType(rgbaColIm, im);
				}
				else
					return 0;
			return res;
		}

		template<>
		inline int readImage(const char * path, Image<RGBAColor> * im)
		{
			std::vector<unsigned char> ptr;
			int w, h, depth;
			int res = readImage(path, &ptr, &w, &h, &depth);
			if (depth != 4) return 0;
			if (res == 1) {
				RGBAColor * ptrCol = (RGBAColor*)&ptr[0];
				//convert raw array to Image
				(*im) = Eigen::Map<Image<RGBAColor>::Base>(ptrCol, h, w);
			}
			return res;
		}

		//--------
		//-- Image Writing
		//--------

		/// Write image to disk, support only unsigned char based type (gray, rgb, rgba)
		template<typename T>
		int writeImage(const char * filename, const Image<T>& im)
		{
			const unsigned char * ptr = (unsigned char*)(im.GetMat().data());
			int depth = sizeof(T) / sizeof(unsigned char);
			std::vector<unsigned char> array(ptr, ptr + im.Width()*im.Height()*depth);
			int w = im.Width(), h = im.Height();
			return writeImage(filename, array, w, h, depth);
		}

		template<typename T>
		int WriteJpg(const char * filename, const Image<T>& im, int quality)
		{
			const unsigned char * ptr = (unsigned char*)(im.GetMat().data());
			const int w = im.Width(), h = im.Height();
			const int depth = sizeof(T) / sizeof(unsigned char);
			std::vector<unsigned char> array(ptr, ptr + w*h*depth);
			return WriteJpg(filename, array, w, h, depth, quality);
		}
	} //namespace image
}  // namespace mvg


#endif // MVG_IMAGE_IMAGE_IO_H_
