#include "mvg/utils/notify.h"
#include "mvg/multiview/homography_parameterization.h"
#include "testing.h"

using namespace mvg::multiview;
using namespace mvg::math;

TEST(Homography2DNormalizedParameterization, Roundtripping) {
	Mat3 h, h_roundtrip;
	Vec8 p; p << 1, 2, 3,
		4, 5, 6,
		7, 8;
	Vec8 p_roundtrip;

	// 构建单应矩阵与参数化之间的循环 
	Homography2DNormalizedParameterization<double>::To(p, &h);

	Homography2DNormalizedParameterization<double>::From(h, &p_roundtrip);

	Homography2DNormalizedParameterization<double>::To(p_roundtrip, &h_roundtrip);

	EXPECT_MATRIX_PROP(h, h_roundtrip, 1.5e-8);
}

TEST(Homography3DNormalizedParameterization, Roundtripping) {
	Mat4 h, h_roundtrip;
	Vec15 p; p << 1, 2, 3, 4,
		5, 6, 7, 8,
		9, 10, 11, 12,
		13, 14, 15;
	Vec15 p_roundtrip;

	// 构建单应矩阵与参数化之间的循环 
	Homography3DNormalizedParameterization<double>::To(p, &h);

	Homography3DNormalizedParameterization<double>::From(h, &p_roundtrip);

	Homography3DNormalizedParameterization<double>::To(p_roundtrip, &h_roundtrip);

	EXPECT_MATRIX_PROP(h, h_roundtrip, 1.5e-8);
}

