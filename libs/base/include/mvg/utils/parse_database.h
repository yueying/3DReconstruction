#ifndef MVG_UTILS_PARSE_DATABASE_H_
#define MVG_UTILS_PARSE_DATABASE_H_

#include "mvg/utils/string_utils.h"
#include "mvg/utils/datasheet.h"

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iterator>

namespace mvg{
	namespace utils{
		/**	对数据进行解析
		 */
		bool parseDatabase(const std::string& m_file_database, std::vector<Datasheet>& vec_database)
		{
			bool createDatabase = false;
			std::ifstream iFilein(m_file_database.c_str());
			if (iFilein.is_open())
			{
				std::string line;
				while (iFilein.good())
				{
					getline(iFilein, line);
					if (!line.empty())
					{
						//std::stringstream sStream( line );
						if (line[0] != '#')
						{
							std::vector<std::string> values;
							split(line, ";", values);
							if (values.size() == 3)
							{
								std::string brand = values[0];
								std::string model = values[1];
								double sensorSize = atof(values[2].c_str());
								vec_database.push_back(Datasheet(brand, model, sensorSize));
							}
						}
					}
				}
				createDatabase = true;
			}
			else
			{
				std::cerr << "Cannot open the database file : "
					<< m_file_database << std::endl;
			}

			return createDatabase;
		}

		/**	根据给定的相机型号得到相关数据
		 */
		bool getInfo(const std::string& m_brand, 
			const std::string& m_model, 
			const std::vector<Datasheet>& vec_database, 
			Datasheet& datasheetription)
		{
			bool existInDatabase = false;

			Datasheet refDatasheet(m_brand, m_model, -1.);
			std::vector<Datasheet>::const_iterator datasheet = std::find(vec_database.begin(), vec_database.end(), refDatasheet);
			if (datasheet != vec_database.end())
			{
				datasheetription = *datasheet;
				existInDatabase = true;
			}

			return existInDatabase;
		}
	}
}
#endif //MVG_UTILS_PARSE_DATABASE_H_

