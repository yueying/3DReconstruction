#ifndef TESTS_TESTING_H_
#define TESTS_TESTING_H_

#include "mvg/math/numeric.h"
#include "gtest/gtest.h"

#define EXPECT_MATRIX_NEAR(a, b, tolerance) \
do { \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  EXPECT_EQ(a.rows(), b.rows()) << "Matrix rows don't match."; \
  EXPECT_EQ(a.cols(), b.cols()) << "Matrix cols don't match."; \
  if (dims_match) { \
    for (int r = 0; r < a.rows(); ++r) { \
      for (int c = 0; c < a.cols(); ++c) { \
        EXPECT_NEAR(a(r, c), b(r, c), tolerance) \
          << "r=" << r << ", c=" << c << "."; \
      } \
    } \
  } \
} while(false);

#define EXPECT_MATRIX_NEAR_ZERO(a, tolerance) \
do { \
  for (int r = 0; r < a.rows(); ++r) { \
    for (int c = 0; c < a.cols(); ++c) { \
      EXPECT_NEAR(0.0, a(r, c), tolerance) \
        << "r=" << r << ", c=" << c << "."; \
    } \
  } \
} while(false);

#define EXPECT_MATRIX_EQ(a, b) \
do { \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  EXPECT_EQ(a.rows(), b.rows()) << "Matrix rows don't match."; \
  EXPECT_EQ(a.cols(), b.cols()) << "Matrix cols don't match."; \
  if (dims_match) { \
    for (int r = 0; r < a.rows(); ++r) { \
      for (int c = 0; c < a.cols(); ++c) { \
        EXPECT_EQ(a(r, c), b(r, c)) \
          << "r=" << r << ", c=" << c << "."; \
      } \
    } \
  } \
} while(false);

// Check that sin(angle(a, b)) < tolerance.
#define EXPECT_MATRIX_PROP(a, b, tolerance) \
do { \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  EXPECT_EQ(a.rows(), b.rows()) << "Matrix rows don't match."; \
  EXPECT_EQ(a.cols(), b.cols()) << "Matrix cols don't match."; \
  if (dims_match) { \
    double c = CosinusBetweenMatrices(a, b); \
    if (c * c < 1) { \
      double s = sqrt(1 - c * c); \
      EXPECT_NEAR(0, s, tolerance); \
	    } \
    } \
} while(false);

template<class TMat>
double CosinusBetweenMatrices(const TMat &a, const TMat &b) {
  return (a.array() * b.array()).sum() / 
      mvg::math::FrobeniusNorm(a) / mvg::math::FrobeniusNorm(b);
}

#endif  // TESTS_TESTING_H_
