/*************************************************************************
 * 文件： datasheet.h
 * 时间： 2015/04/29 16:29
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 用于存放相机品牌，型号及对应焦距值
 *************************************************************************/
#ifndef MVG_UTILS_DATASHEET_H_
#define MVG_UTILS_DATASHEET_H_

#include "mvg/utils/string_utils.h"
#include <iterator>

namespace mvg{
	namespace utils{
		// Database structure
		struct Datasheet
		{
			Datasheet()
			{}

			Datasheet(const std::string& brand,
				const std::string& model,
				const double& sensorSize) :
				_brand(brand),
				_model(model),
				_sensorSize(sensorSize)
			{}

			bool operator==(const Datasheet& ds) const
			{
				bool is_equal = false;
				std::vector<std::string> vec_brand;
				split(ds._brand, " ", vec_brand);
				std::string brandlower = _brand;

				for (int index = 0; index < brandlower.length(); index++)
				{
					brandlower[index] = tolower(brandlower[index]);
				}

				for (std::vector<std::string>::const_iterator iter_brand = vec_brand.begin();
					iter_brand != vec_brand.end();
					iter_brand++)
				{
					std::string brandlower2 = *iter_brand;
					for (int index = 0; index < brandlower2.length(); index++)
					{
						brandlower2[index] = tolower(brandlower2[index]);
					}
					//std::cout << brandlower << "\t" << brandlower2 << std::endl;
					if (brandlower.compare(brandlower2) == 0)
					{

						std::vector<std::string> vec_model1;
						split(ds._model, " ", vec_model1);
						std::vector<std::string> vec_model2;
						split(_model, " ", vec_model2);
						bool is_all_find = true;
						for (std::vector<std::string>::const_iterator iter_model1 = vec_model1.begin();
							iter_model1 != vec_model1.end();
							iter_model1++)
						{
							bool hasDigit = false;
							for (std::string::const_iterator c = (*iter_model1).begin(); c != (*iter_model1).end(); ++c)
							{
								if (isdigit(*c))
								{
									hasDigit = true;
									break;
								}
							}
							if (hasDigit)
							{
								std::string modellower1 = *iter_model1;
								for (int index = 0; index < modellower1.length(); index++)
								{
									modellower1[index] = tolower(modellower1[index]);
								}
								bool is_find = false;
								for (std::vector<std::string>::const_iterator iter_model2 = vec_model2.begin();
									iter_model2 != vec_model2.end();
									iter_model2++)
								{
									std::string modellower2 = *iter_model2;
									for (size_t index = 0; index < modellower2.length(); index++)
									{
										modellower2[index] = tolower(modellower2[index]);
									}
									if (modellower2.compare(modellower1) == 0)
									{
										is_find = true;
									}
								}
								if (!is_find)
								{
									is_all_find = false;
									break;
								}
							}
						}
						if (is_all_find)
							is_equal = true;
					}
				}
				return is_equal;
			}

			std::string _brand;
			std::string _model;
			double _sensorSize;
		};

	}
}
#endif // MVG_UTILS_DATASHEET_H_

