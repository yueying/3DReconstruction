#include <iostream>
#include <cstdlib>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

#include "fblib/utils/svg_drawer.h"
using namespace fblib::utils;

int main(int argc, char * argv[])
{
	// Simple usage:
	{
		SvgDrawer svgSurface; //Create a svg object
		// Add some drawing
		double S = 20.;
		for (double i = 0; i < 3.14 * 2; i += .4) {
			double ax = cos(i)*S + S;
			double ay = sin(i)*S + S;
			double bx = cos(i + 3.14 / 4.)*S + S;
			double by = sin(i + 3.14 / 4.)*S + S;
			// Drawing are controlled by function
			//  SvgStyle that configure the drawing option
			svgSurface.drawLine(ax, ay, bx, by,
				SvgStyle().stroke("blue", 1));
		}
		//Export the SVG stream to a file
		std::string file_name = "FirstExample.svg";
		std::ofstream svg_file(file_name.c_str());
		svg_file << svgSurface.closeSvgFile().str();
		svg_file.close();
	}

	// Other drawing primitive:
  {
	  SvgDrawer svgSurface(20, 20); //Create a svg object
	  // Add some drawing

	  svgSurface.drawCircle(10, 10, 4,
		  SvgStyle().stroke("red", 1).fill("blue").tooltip("Hello"));

	  svgSurface.drawSquare(4, 4, 12, SvgStyle().stroke("black"));

	  svgSurface.drawText(8, 11, 6.f, "H", "green");

	  //Export the SVG stream to a file
	  std::string file_name = "SecondExample.svg";
	  std::ofstream svg_file(file_name.c_str());
	  svg_file << svgSurface.closeSvgFile().str();
	  svg_file.close();
  }

	// Draw a cardiod with the svg polyline:
	// http://en.wikipedia.org/wiki/Cardioid
  {
	  // Pre-compute (x,y) curve points
	  size_t points_num = 120;
	  std::vector<float> vec_x(points_num, 0.f), vec_y(points_num, 0.f);
	  double S = 20.;
	  for (size_t i = 0; i < points_num; ++i) {
		  const double theta = i * 2 * M_PI / points_num;
		  //-- Cardioid equation
		  vec_x[i] = (3 * S + S*(2.*sin(theta) - (sin(2.*theta))));
		  vec_y[i] = (2 * S - S*(2.*cos(theta) - (cos(2.*theta))));
	  }
	  // Create a svg surface and add the cardiod polyline
	  SvgDrawer svgSurface(6 * S, 6 * S); //Create a svg object
	  svgSurface.drawPolyline(
		  vec_x.begin(), vec_x.end(),
		  vec_y.begin(), vec_y.end(),
		  SvgStyle().stroke("blue", 2));

	  //Export the SVG stream to a file
	  std::string file_name = "ThirdExample.svg";
	  std::ofstream svg_file(file_name.c_str());
	  svg_file << svgSurface.closeSvgFile().str();
	  svg_file.close();
  }
	return EXIT_SUCCESS;
}
