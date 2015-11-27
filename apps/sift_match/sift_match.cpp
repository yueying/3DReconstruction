#include "mvg/image/image.h"
#include "mvg/image/image_io.h"
#include "mvg/image/image_concat.h"
#include "mvg/image/image_drawing.h"
#include "mvg/image/image_converter.h"
#include "mvg/feature/feature.h"
#include "mvg/feature/matcher_brute_force.h"
#include "mvg/feature/matcher_kdtree_flann.h"
#include "mvg/feature/matching_filters.h"
#include "mvg/feature/metric.h"
#include "mvg/feature/indexed_match.h"
#include "mvg/feature/two_view_matches.h"

#include "mvg/feature/sift.hpp"
#include "mvg/utils/file_system.h"

#include "mvg/utils/svg_drawer.h"

#include <string>
#include <iostream>
#include <vector>


using namespace mvg::feature;
using namespace mvg::image;
using namespace std;

namespace mvg {
	namespace utils {
		std::string MVG_GLOBAL_SRC_DIR;
	}
}
using namespace mvg::utils;
int main() {

	Image<RGBColor> image;
	string left_image_name = MVG_GLOBAL_SRC_DIR
		+ "/imageData/StanfordMobileVisualSearch/Ace_0.png";
	string right_image_name = MVG_GLOBAL_SRC_DIR
		+ "/imageData/StanfordMobileVisualSearch/Ace_1.png";

	Image<unsigned char> left_image, right_image;
	readImage(left_image_name.c_str(), &left_image);
	readImage(right_image_name.c_str(), &right_image);

	// Define the used descriptor (SIFT : 128 float value)
	typedef unsigned char descType;
	typedef Descriptor<descType, 128> SIFTDescriptor;

	// Prepare vector to store detected feature and associated descriptor
	vector<ScalePointFeature> left_features, right_features;
	vector<SIFTDescriptor > left_descriptors, right_descriptors;
	// Call SIFT detector
	bool is_zoom = false;
	bool is_root_sift = true;
	SIFTDetector(left_image, left_features, left_descriptors, is_zoom, is_root_sift);
	SIFTDetector(right_image, right_features, right_descriptors, is_zoom, is_root_sift);

	// Show both images side by side
	{
		Image<unsigned char> concat;
		concatHorizontal(left_image, right_image, concat);
		string out_filename = "00_images.jpg";
		writeImage(out_filename.c_str(), concat);
	}

	//- Draw features on the two image (side by side)
  {
	  Image<unsigned char> concat;
	  concatHorizontal(left_image, right_image, concat);

	  //-- Draw features :
	  for (size_t i = 0; i < left_features.size(); ++i)  {
		  const ScalePointFeature & left_img = left_features[i];
		  DrawCircle(left_img.x(), left_img.y(), left_img.scale(), 255, &concat);
	  }
	  for (size_t i = 0; i < right_features.size(); ++i)  {
		  const ScalePointFeature & right_img = right_features[i];
		  DrawCircle(right_img.x() + left_image.Width(), right_img.y(), right_img.scale(), 255, &concat);
	  }
	  string out_filename = "01_features.jpg";
	  writeImage(out_filename.c_str(), concat);
  }

	//-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
	std::vector<IndexedMatch> vec_putative_matches;
	{
		// Define the matcher
		//  and the used metric (Squared L2)
		typedef SquaredEuclideanDistanceVectorized<SIFTDescriptor::bin_type> Metric;
		// Brute force matcher is defined as following:
		typedef ArrayMatcherBruteForce<SIFTDescriptor::bin_type, Metric> MatcherT;
		// ANN matcher could be defined as follow:
		//typedef ArrayMatcher_Kdtree_nanoflann<SIFTDescriptor::bin_type,
		//  Metric // Metric to compare two descriptor
		//  > MatcherT;

		// Distance ratio squared due to squared metric
		GetPutativesMatches<SIFTDescriptor, MatcherT>(left_descriptors, right_descriptors, Square(0.6), vec_putative_matches);

		// Draw correspondences after Nearest Neighbor ratio filter
		SvgDrawer svg_stream(left_image.Width() + right_image.Width(), max(left_image.Height(), right_image.Height()));
		svg_stream.drawImage(left_image_name, left_image.Width(), left_image.Height());
		svg_stream.drawImage(right_image_name, right_image.Width(), right_image.Height(), left_image.Width());
		for (size_t i = 0; i < vec_putative_matches.size(); ++i) {
			//Get back linked feature, draw a circle and link them by a line
			const ScalePointFeature & L = left_features[vec_putative_matches[i]._i];
			const ScalePointFeature & R = right_features[vec_putative_matches[i]._j];
			svg_stream.drawLine(L.x(), L.y(), R.x() + left_image.Width(), R.y(), SvgStyle().stroke("green", 2.0));
			svg_stream.drawCircle(L.x(), L.y(), L.scale(), SvgStyle().stroke("yellow", 2.0));
			svg_stream.drawCircle(R.x() + left_image.Width(), R.y(), R.scale(), SvgStyle().stroke("yellow", 2.0));
		}
		string out_filename = "02_siftMatches.svg";
		ofstream svg_file(out_filename.c_str());
		svg_file << svg_stream.closeSvgFile().str();
		svg_file.close();
	}

	// Display some statistics
	std::cout << left_features.size() << " Features on image A" << std::endl
		<< right_features.size() << " Features on image B" << std::endl
		<< vec_putative_matches.size() << " matches after matching with Distance Ratio filter" << std::endl;

	return EXIT_SUCCESS;
}
