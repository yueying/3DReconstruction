#include "mvg/utils/cmd_line.h"
#include "mvg/utils/parse_database.h"

using namespace mvg::utils;
int main(int argc, char ** argv)
{
	CmdLine cmd;
	std::cout << argc << std::endl;
	std::string m_file_database;
	std::string m_brand;
	std::string m_model;

	cmd.add(make_option('i', m_file_database, "ifileDB"));
	cmd.add(make_option('b', m_brand, "iBrand"));
	cmd.add(make_option('m', m_model, "iModel"));

	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Look if the given references exist in the camera database file.\nUsage: " << argv[0] << ' '
			<< "[-i|--DatabaseFile path] "
			<< "[-b|--Brand ] "
			<< "[-m|--Model] "
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << " You called : " << std::endl
		<< argv[0] << std::endl
		<< "--DatabaseFile " << m_file_database << std::endl
		<< "--Brand " << m_brand << std::endl
		<< "--Model " << m_model << std::endl;

	std::vector<Datasheet> vec_database;
	Datasheet datasheet;

	if (!parseDatabase(m_file_database, vec_database))
	{
		std::cout << "Database creation failure from the file : " << m_file_database << std::endl;
		return EXIT_FAILURE;
	}

	if (!getInfo(m_brand, m_model, vec_database, datasheet))
	{
		std::cout << "The camera " << m_model << " doesn't exist in the database" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Result : " << std::endl << datasheet._brand << "\t" << datasheet._model << "\t" << datasheet._sensorSize << std::endl;

	getchar();
	return EXIT_SUCCESS;
}

