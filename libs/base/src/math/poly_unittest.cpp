#include "fblib/math/numeric.h"
#include "fblib/math/poly.h"
#include "testing.h"

using namespace fblib::math;
// 将方程展开成多项式的形式，找出对应系数
//
//   (x - a)(x - b)(x - c) == 0  展开为
//   
//   x^3 - (c+b+a) * x^2 + (a*b+(b+a)*c) * x - a*b*c = 0.
//           = p               = q              = r
void CoeffsForCubicZeros(double a, double b, double c,
                    double *p, double *q, double *r) {
  *p = -(c + b + a);
  *q = (a * b + (b + a) * c);
  *r = -a * b * c;
}

// 将方程展开成多项式的形式，找出对应系数
//
//   (x - a)(x - b)(x - c)(x - d) == 0 展开为
//
//   x^4 - (d+c+b+a) * x^3 + (d*(c+b+a) + a*b+(b+a)*c) * x^2
//   - (d*(a*b+(b+a)*c)+a*b*c) * x + a*b*c*d = 0.
void CoeffsForQuarticZeros(double a, double b, double c, double d,
                    double *p, double *q, double *r, double *s) {
  *p = -(d + c + b + a);
  *q = (d * (c + b + a) + a * b + (b + a) * c);
  *r = -(d * (a * b + (b + a) * c) + a * b * c);
  *s = a * b * c *d;
}

TEST(Poly, SolveCubicPolynomial) {
  double a, b, c, aa, bb, cc;
  double p, q, r;

  a = 1; b = 2; c = 3;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  EXPECT_EQ(3, SolveCubicPolynomial(p, q, r, &aa, &bb, &cc));
  EXPECT_NEAR(a, aa, 1e-10);
  EXPECT_NEAR(b, bb, 1e-10);
  EXPECT_NEAR(c, cc, 1e-10);

  a = 0; b = 1; c = 3;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  EXPECT_EQ(3, SolveCubicPolynomial(p, q, r, &aa, &bb, &cc));
  EXPECT_NEAR(a, aa, 1e-10);
  EXPECT_NEAR(b, bb, 1e-10);
  EXPECT_NEAR(c, cc, 1e-10);

  a = -10; b = 0; c = 1;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  EXPECT_EQ(3, SolveCubicPolynomial(p, q, r, &aa, &bb, &cc));
  EXPECT_NEAR(a, aa, 1e-10);
  EXPECT_NEAR(b, bb, 1e-10);
  EXPECT_NEAR(c, cc, 1e-10);

  a = -8; b = 1; c = 3;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  EXPECT_EQ(3, SolveCubicPolynomial(p, q, r, &aa, &bb, &cc));
  EXPECT_NEAR(a, aa, 1e-10);
  EXPECT_NEAR(b, bb, 1e-10);
  EXPECT_NEAR(c, cc, 1e-10);

  a = 28; b = 28; c = 105;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  EXPECT_EQ(3, SolveCubicPolynomial(p, q, r, &aa, &bb, &cc));
  EXPECT_NEAR(a, aa, 1e-10);
  EXPECT_NEAR(b, bb, 1e-10);
  EXPECT_NEAR(c, cc, 1e-10);
}

