#include "fblib/feature/estimator_line_kernel.h"
#include "fblib/feature/estimator_min_median.h"
#include "fblib/feature/score_evaluator.h"

#include "fblib/math/numeric.h"

#include "testing.h"

using namespace fblib::feature;
using namespace fblib::math;

static const double kExpectedPrecision = 1e-9;

template<typename Kernel>
void EvalInlier(const Kernel &kernel, const typename Kernel::Model &model,
	double threshold, std::vector<size_t> * vec_inliers)
{
	ScorerEvaluator<Kernel> scorer(threshold);
	std::vector<size_t> vec_index(kernel.NumSamples());
	for (size_t i = 0; i < kernel.NumSamples(); ++i)
		vec_index[i] = i;

	scorer.Score(kernel, model, vec_index, &(*vec_inliers));
}

// 测试没有外点的情况
TEST(LMedsLineFitter, OutlierFree) {

	Mat2X xy(2, 5);
	// y = 2x + 1
	xy << 1, 2, 3, 4, 5,
		3, 5, 7, 9, 11;

	// 基础线性估计内核
	LineKernel kernel(xy);

	// 使用LMeds算法对模型进行鲁棒性估计
	Vec2 model;
	double threshold = std::numeric_limits<double>::infinity();
	double best_median = LeastMedianOfSquares(kernel, &model, &threshold);
	EXPECT_NEAR(2.0, model[1], kExpectedPrecision);
	EXPECT_NEAR(1.0, model[0], kExpectedPrecision);
	EXPECT_NEAR(0.0, best_median, kExpectedPrecision);
	EXPECT_NEAR(0.0, threshold, kExpectedPrecision);
	//计算哪些点是内点
	std::vector<size_t> vec_inliers;
	EvalInlier(kernel, model, kExpectedPrecision, &vec_inliers);
	EXPECT_EQ(5, vec_inliers.size());
}

// 测试包含内外点的数据
TEST(LMedsLineFitter, OneOutlier) {

	Mat2X xy(2, 6);
	// y = 2x + 1 数据中添加外点
	xy << 1, 2, 3, 4, 5, 100, // outlier!
		3, 5, 7, 9, 11, -123; // outlier!

	LineKernel kernel(xy);

	Vec2 model;
	double threshold = std::numeric_limits<double>::infinity();
	double best_median = LeastMedianOfSquares(kernel, &model, &threshold);
	EXPECT_NEAR(2.0, model[1], kExpectedPrecision);
	EXPECT_NEAR(1.0, model[0], kExpectedPrecision);
	EXPECT_NEAR(0.0, best_median, kExpectedPrecision);
	EXPECT_NEAR(0.0, threshold, kExpectedPrecision);
	//计算哪些点是内点
	std::vector<size_t> vec_inliers;
	EvalInlier(kernel, model, kExpectedPrecision, &vec_inliers);
	EXPECT_EQ(5, vec_inliers.size());
}

// 临界测试，如果进行鲁棒性估计的点太少的情况下
TEST(LMedsLineFitter, TooFewPoints) {

	Mat2X xy(2, 1);
	xy << 1,
		3;   // y = 2x + 1 with x = 1
	LineKernel kernel(xy);

	Vec2 model;
	double threshold = std::numeric_limits<double>::infinity();
	double best_median = LeastMedianOfSquares(kernel, &model, &threshold);
	//没有内点
	EXPECT_EQ(best_median, std::numeric_limits<double>::max());
}

// From a GT model :
//  Compute a list of point that fit the model.
//  Add white noise to given amount of points in this list.
//  Check that the number of inliers and the model are correct.
TEST(LMedsLineFitter, RealisticCase) {

	const int NbPoints = 30;
	const int inlierPourcentAmount = 30; //works with 40
	Mat2X xy(2, NbPoints);

	Vec2 GTModel; // y = 2x + 1
	GTModel << -2.0, 6.3;

	//-- Build the point list according the given model
	for (int i = 0; i < NbPoints; ++i)  {
		xy.col(i) << i, (double)i*GTModel[1] + GTModel[0];
	}

	//-- Add some noise (for the asked percentage amount)
	int nbPtToNoise = (int)NbPoints*inlierPourcentAmount / 100.0;
	vector<size_t> vec_samples; // Fit with unique random index
	UniformSample(nbPtToNoise, NbPoints, &vec_samples);
	for (size_t i = 0; i < vec_samples.size(); ++i)
	{
		const size_t randomIndex = vec_samples[i];
		//Additive random noise
		xy.col(randomIndex) << xy.col(randomIndex)(0) + rand() % 2 - 3,
			xy.col(randomIndex)(1) + rand() % 8 - 6;
	}

	LineKernel kernel(xy);

	Vec2 model;
	double threshold = std::numeric_limits<double>::infinity();
	double best_median = LeastMedianOfSquares(kernel, &model, &threshold);
	EXPECT_NEAR(-2.0, model[0], kExpectedPrecision);
	EXPECT_NEAR(6.3, model[1], kExpectedPrecision);
	//Compute which point are inliers (error below threshold)
	std::vector<size_t> vec_inliers;
	EvalInlier(kernel, model, threshold, &vec_inliers);
	EXPECT_TRUE(vec_inliers.size()>0);
	EXPECT_EQ(NbPoints - nbPtToNoise, vec_inliers.size());
}
