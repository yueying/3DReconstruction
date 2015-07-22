#include "testing.h"

#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>

#include "fblib/feature/features.h"
using namespace fblib::feature;

const int kCardDescs = 12;
const int kDescLength = 128;
typedef Descriptor<float, kDescLength> DescT;

TEST(descriptorIO, ASCII) {
  // 创建描述子的输入
  std::vector<DescT > vec_descs;
  for(int i = 0; i < kCardDescs; ++i)  {
    DescT desc;
    for (int j = 0; j < kDescLength; ++j)
      desc[j] = i*kDescLength+j;
    vec_descs.push_back(desc);
  }

  //保存到文件
  SaveDescsToFile("tempDescs.desc", vec_descs);

  //读取描述子进行比较
  std::vector<DescT > vec_descs_read;
  LoadDescsFromFile("tempDescs.desc", vec_descs_read);
  EXPECT_EQ(kCardDescs, vec_descs_read.size());

  for(int i = 0; i < kCardDescs; ++i) {
    for (int j = 0; j < kDescLength; ++j)
      EXPECT_EQ(vec_descs[i][j], vec_descs_read[i][j]);
  }
}

//测试二进制的描述子的导入导出
TEST(descriptorIO, BINARY) {
  //创建描述子的输入序列
  std::vector<DescT > vec_descs;
  for(int i = 0; i < kCardDescs; ++i)
  {
    DescT desc;
    for (int j = 0; j < kDescLength; ++j)
      desc[j] = i*kDescLength+j;
    vec_descs.push_back(desc);
  }

  //保存到文件
  SaveDescsToBinFile("tempDescsBin.desc", vec_descs);

  //读取进行比较
  std::vector<DescT > vec_descs_read;
  LoadDescsFromBinFile("tempDescsBin.desc", vec_descs_read);
  EXPECT_EQ(kCardDescs, vec_descs_read.size());

  for(int i = 0; i < kCardDescs; ++i) {
    for (int j = 0; j < kDescLength; ++j)
      EXPECT_EQ(vec_descs[i][j], vec_descs_read[i][j]);
  }
}
