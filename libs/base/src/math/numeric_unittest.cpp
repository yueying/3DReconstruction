#include <iostream>
#include "testing.h"

#include "mvg/math/numeric.h"
using namespace mvg::math;

// 验证输出
TEST ( TinyMatrix, print )
{
  Mat3 test_matrix = Mat3::Identity();
  std::cout << test_matrix;
}

TEST ( TinyMatrix, checkIdentity )
{
  Mat3 test_matrix, expected;

  // 构建单位矩阵，与eigen中给出的单位阵进行比较
  expected.fill(0);
  expected(0,0) = expected(1,1) = expected(2,2) = 1.0;

  test_matrix.setIdentity();
  std::cout << std::endl << test_matrix;
  //进行两个矩阵比较
  EXPECT_MATRIX_NEAR( expected, test_matrix, 1e-8);
}

TEST(TinyMatrix, checkRotationAround)
{
	Mat3 x, y, z, expected;
	// 绕X轴转动90度
	x << 1.0000, 0, 0,
		0, 0.0000, -1.0000,
		0, 1.0000, 0.0000;
	expected = RotationAroundX(M_PI/2);
	std::cout << std::endl << expected;
	EXPECT_MATRIX_NEAR(expected, x, 1e-8);
}

TEST ( TinyMatrix, product )
{
  Mat3 a, b, expected;

  a(0,0) = 1.0; a(0,1) = 2.0; a(0,2) = 3.0;
  a(1,0) = 4.0; a(1,1) = 5.0; a(1,2) = 6.0;
  a(2,0) = 7.0; a(2,1) = 8.0; a(2,2) = 9.0;

  b(0,0) = 10.0; b(0,1) = 11.0; b(0,2) = 12.0;
  b(1,0) = 13.0; b(1,1) = 14.0; b(1,2) = 15.0;
  b(2,0) = 16.0; b(2,1) = 17.0; b(2,2) = 18.0;

  Mat3 res_axb = a*b;
 
  Mat3 expected_res_axb;
  expected_res_axb << 84.0,90.0,96.0,
	  201.0,216.0,231.0,
	  318.0,342.0,366.0;

  Mat3 res_bxa = b*a;
  Mat3 expected_res_bxa;
  expected_res_bxa << 138, 171, 204,
	  174, 216, 258,
	  210, 261, 312;

  //-- Tests
  EXPECT_MATRIX_NEAR( expected_res_axb, res_axb, 1e-8);
  EXPECT_MATRIX_NEAR( expected_res_bxa, res_bxa, 1e-8);
}

TEST(TinyMatrix, LookAt) {
  // 简单的正交验证
  Vec3 e; e[0]= 1; e[1] = 2; e[2] = 3;
  Mat3 R = LookAt(e);//这个R是旋转矩阵，则R为正交矩阵，R与R的转置相乘为单位阵
  Mat3 I = Mat3::Identity();
  Mat3 RRT = R*R.transpose();
  Mat3 RTR = R.transpose()*R;

  EXPECT_MATRIX_NEAR(I, RRT, 1e-15);
  EXPECT_MATRIX_NEAR(I, RTR, 1e-15);
}

TEST(Numeric, extractColumns) {
  Mat2X A(2, 5);
  A << 1, 2, 3, 4, 5,
       6, 7, 8, 9, 10;
  Vec2i columns; columns << 0, 2;
  Mat2X extracted = extractColumns(A, columns);
  EXPECT_NEAR(1, extracted(0,0), 1e-15);
  EXPECT_NEAR(3, extracted(0,1), 1e-15);
  EXPECT_NEAR(6, extracted(1,0), 1e-15);
  EXPECT_NEAR(8, extracted(1,1), 1e-15);
}

TEST(Numeric, MeanAndVarianceAlongRows) {
  int n = 4;
  Mat points(2,n);
  points << 0, 0, 1, 1,
    0, 2, 1, 3;

  Vec mean, variance;
  MeanAndVarianceAlongRows(points, &mean, &variance);

  EXPECT_NEAR(0.5, mean(0), 1e-8);
  EXPECT_NEAR(1.5, mean(1), 1e-8);
  EXPECT_NEAR(0.25, variance(0), 1e-8);
  EXPECT_NEAR(1.25, variance(1), 1e-8);
}

