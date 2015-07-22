#include "fblib/image/image.h"
#include "testing.h"

#include <iostream>

using namespace std;
using namespace fblib::image;

TEST(Image, Basis)
{
  //创建灰度图像Image<unsigned char>
  Image<unsigned char> img_gray(10,10);
  img_gray(1,1) = 1; //设定像素值
  img_gray(2,2) = 2;
  img_gray(5,0) = 2;

  cout << img_gray << endl << endl;
  // 得到指向图像数据的原始指针
  const unsigned char * ptr = img_gray.data();
  const_cast<unsigned char*>(ptr)[0] = 2;
  fill(((unsigned char*)ptr+9*10),((unsigned char*)ptr+10*10),2);
  cout << "After" << endl << img_gray;

  // 通过复制构造函数进行构造
  Image<unsigned char> img_gray2(img_gray);

  // 构件矩阵通过拷贝函数进行构造
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix(5,5);
  Image<unsigned char> img_gray3 = matrix;

  // 从图像中返回矩阵
  matrix = img_gray.GetMat();

  // 矩阵复制
  Image<unsigned char> img_gray4;
  img_gray4 = matrix;

  Image<unsigned char> img_gray5;
  img_gray5 = img_gray;

  // 创建RGB图像
  Image<RGBColor> imaRGB(10,10);
  imaRGB(0,0) = RGBColor(0,1,2);

  // 创建RGBA图像
  Image<RGBAColor> imaRGBA(10,10);
  imaRGBA(0,0) = RGBAColor(0,1,2,1);
  imaRGBA(1,0) = RGBAColor(1,1,1);
}

TEST(Image, PixelTypes)
{
  RGBColor  a(BLACK);
  // RGBColor  c(0); // Not accepted because can cause bad pixel affectation value (mixed type...)
  // The following issue must used : (at your own risk)
  RGBColor  b(static_cast<unsigned char>(0));
  RGBAColor d(BLACK);
}
