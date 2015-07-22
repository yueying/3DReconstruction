#include <cstdlib>

#include "fblib/sfm/sfm_global_engine.h"
using namespace fblib::sfm;

#include "fblib/utils/cmd_line.h"
#include "fblib/utils/file_system.h"
#include "fblib/utils/timer.h"

int main(int argc, char **argv)
{
	using namespace std;
	std::cout << std::endl
		<< "-----------------------------------------------------------\n"
		<< "Global Structure from Motion:\n"
		<< "-----------------------------------------------------------\n"
		<< "Open Source implementation of the paper:\n"
		<< "\"Global Fusion of Relative Motions for "
		<< "Robust, Accurate and Scalable Structure from Motion.\"\n"
		<< "Pierre Moulon, Pascal Monasse and Renaud Marlet. "
		<< " ICCV 2013." << std::endl
		<< "------------------------------------------------------------"
		<< std::endl;


	CmdLine cmd;

	std::string image_dir;
	std::string matches_dir;
	std::string out_dir = "";
	bool is_colored_point_cloud = false;

	cmd.add(make_option('i', image_dir, "imadir"));
	cmd.add(make_option('m', matches_dir, "matchdir"));
	cmd.add(make_option('o', out_dir, "outdir"));
	cmd.add(make_option('c', is_colored_point_cloud, "coloredPointCloud"));

	try {
		if (argc == 1) throw std::string("Invalid parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Usage: " << argv[0] << '\n'
			<< "[-i|--imadir path]\n"
			<< "[-m|--matchdir path]\n"
			<< "[-o|--outdir path]\n"
			<< "[-c|--coloredPointCloud 0(default) or 1]\n"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	if (out_dir.empty())  {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}

	if (!fblib::utils::folder_exists(out_dir))
		fblib::utils::folder_create(out_dir);

	fblib::utils::Timer timer;
	timer.Start();
	GlobalReconstructionEngine to3DEngine(image_dir,
		matches_dir,
		out_dir,
		true);

	if (to3DEngine.Process())
	{
		std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.Stop() << std::endl;

		//-- Compute color if requested
		const reconstructorHelper & reconstructorHelperRef = to3DEngine.refToReconstructorHelper();
		std::vector<Vec3> vec_tracks_color;
		if (is_colored_point_cloud)
		{
			// Compute the color of each track
			to3DEngine.ColorizeTracks(to3DEngine.getTracks(), vec_tracks_color);
		}

		//-- Export computed data to disk
		reconstructorHelperRef.exportToPlyFile(
			fblib::utils::create_filespec(out_dir, "FinalColorized", ".ply"),
			is_colored_point_cloud ? &vec_tracks_color : NULL);

		// Export to fblib format
		std::cout << std::endl << "Export 3D scene to fblib format" << std::endl
			<< " -- Point cloud color: " << (is_colored_point_cloud ? "ON" : "OFF") << std::endl;

		reconstructorHelperRef.ExportToOpenMVGFormat(
			fblib::utils::folder_append_separator(out_dir) + "SfM_output",
			to3DEngine.getFilenamesVector(),
			image_dir,
			to3DEngine.getImagesSize(),
			to3DEngine.getTracks(),
			is_colored_point_cloud ? &vec_tracks_color : NULL,
			true,
			std::string("generated by the Global OpenMVG Calibration Engine")
			);

		return EXIT_SUCCESS;
	}

	return EXIT_FAILURE;
}
