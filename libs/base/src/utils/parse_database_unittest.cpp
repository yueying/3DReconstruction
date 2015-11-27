#include <string>
#include "testing.h"

#include "mvg/utils/parse_database.h"
#include "mvg/utils/file_system.h"

namespace mvg {
	namespace utils {
		extern std::string MVG_GLOBAL_SRC_DIR;
	}
}

using namespace mvg::utils;

TEST(Matching, ParseDatabaseSD900)
{
	std::vector<Datasheet> vec_database;
	Datasheet datasheet;
	std::string m_file_database = create_filespec(MVG_GLOBAL_SRC_DIR, "data/cameraGenerated.txt");
	std::string m_model = "Canon PowerShot SD900";
	std::string m_brand = "Canon";

	EXPECT_TRUE(parseDatabase(m_file_database, vec_database));
	EXPECT_TRUE(getInfo(m_brand, m_model, vec_database, datasheet));
	EXPECT_EQ("Canon", datasheet._brand);
	EXPECT_EQ("Canon PowerShot SD900", datasheet._model);
	EXPECT_EQ(7.11, datasheet._sensorSize);
}

TEST(Matching, ParseDatabaseA710_IS)
{
	std::vector<Datasheet> vec_database;
	Datasheet datasheet;
	std::string m_file_database = create_filespec(MVG_GLOBAL_SRC_DIR, "data/cameraGenerated.txt");
	std::string m_model = "Canon PowerShot A710 IS";
	std::string m_brand = "Canon";

	EXPECT_TRUE(parseDatabase(m_file_database, vec_database));
	EXPECT_TRUE(getInfo(m_brand, m_model, vec_database, datasheet));
	EXPECT_EQ("Canon", datasheet._brand);
	EXPECT_EQ("Canon PowerShot A710 IS", datasheet._model);
	EXPECT_EQ(5.75, datasheet._sensorSize);
}

TEST(Matching, ParseDatabaseNotExist)
{
	std::vector<Datasheet> vec_database;
	Datasheet datasheet;
	std::string m_file_database = create_filespec(MVG_GLOBAL_SRC_DIR, "data/cameraGenerated.txt");
	std::string m_model = "NotExistModel";
	std::string m_brand = "NotExistBrand";

	EXPECT_TRUE(parseDatabase(m_file_database, vec_database));
	EXPECT_FALSE(getInfo(m_brand, m_model, vec_database, datasheet));
}


TEST(Matching, ParseDatabaseCanon_EOS_550D)
{
	std::vector<Datasheet> vec_database;
	Datasheet datasheet;
	std::string m_file_database = create_filespec(MVG_GLOBAL_SRC_DIR, "data/cameraGenerated.txt");
	std::string m_model = "Canon EOS 550D";
	std::string m_brand = "Canon";

	EXPECT_TRUE(parseDatabase(m_file_database, vec_database));
	EXPECT_TRUE(getInfo(m_brand, m_model, vec_database, datasheet));
	EXPECT_EQ(22.3, datasheet._sensorSize);
}

TEST(Matching, ParseDatabaseCanon_EOS_5D_Mark_II)
{
	std::vector<Datasheet> vec_database;
	Datasheet datasheet;
	std::string m_file_database = create_filespec(MVG_GLOBAL_SRC_DIR, "data/cameraGenerated.txt");
	std::string m_model = "Canon EOS 5D Mark II";
	std::string m_brand = "Canon";

	EXPECT_TRUE(parseDatabase(m_file_database, vec_database));
	EXPECT_TRUE(getInfo(m_brand, m_model, vec_database, datasheet));
	EXPECT_EQ(36, datasheet._sensorSize);
}

TEST(Matching, ParseDatabaseCanon_EOS_1100D)
{
	std::vector<Datasheet> vec_database;
	Datasheet datasheet;
	std::string m_file_database = create_filespec(MVG_GLOBAL_SRC_DIR, "data/cameraGenerated.txt");
	std::string m_model = "Canon EOS 1100D";
	std::string m_brand = "Canon";

	EXPECT_TRUE(parseDatabase(m_file_database, vec_database));
	EXPECT_TRUE(getInfo(m_brand, m_model, vec_database, datasheet));
	EXPECT_EQ(22.2, datasheet._sensorSize);
}
