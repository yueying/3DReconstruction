#include <iostream>
#include "fblib/image/image.h"
#include "fblib/image/convolve.h"

using namespace fblib::image;

int main()
{
	Image<float> im(10, 10);
	im.fill(1);
	Image<RGBfColor> blurred_and_derivatives;
	BlurredImageAndDerivativesChannels(im, 1.0, &blurred_and_derivatives);
	std::cout << blurred_and_derivatives(5, 5)(0);
	getchar();
	return 0;
}