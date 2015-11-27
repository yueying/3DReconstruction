#include <testing.h>

using namespace std;
//TODO:后期添加预处理命令，直接通过cmake设置预处理路径
namespace mvg { namespace utils {
	std::string MVG_GLOBAL_SRC_DIR;
  }
}

int main(int argc, char **argv)
{
	//testing::GTEST_FLAG(break_on_failure) = true;
	testing::InitGoogleTest(&argc, argv);

	if (argc>1) mvg::utils::MVG_GLOBAL_SRC_DIR=std::string(argv[1]);

	int code = RUN_ALL_TESTS();
	getchar();
	return code;
}
