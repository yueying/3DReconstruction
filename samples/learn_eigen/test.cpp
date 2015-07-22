#include <iostream>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

void test1()
{
	Matrix2d a;
	std::cout << a.stride() << std::endl;

	a << 1, 2,
		3, 4;
	Vector3d v(1, 2, 3);
	std::cout << "a * 2.5 =\n" << a * 2.5 << std::endl;
	std::cout << "0.1 * v =\n" << 0.1 * v << std::endl;
	std::cout << "Doing v *= -2;" << std::endl;
	v *= -2;
	std::cout << "Now v =\n" << v << std::endl;
}

int main()
{
	typedef Eigen::Matrix<double,3, 4> Mat34;
	std::vector<Mat34> Ps(5);
	test1();
	getchar();
	return 0;
}