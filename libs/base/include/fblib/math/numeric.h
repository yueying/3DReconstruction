#ifndef FBLIB_MATH_NUMERIC_H_
#define FBLIB_MATH_NUMERIC_H_

// http://eigen.tuxfamily.org/dox-devel/QuickRefPage.html
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SVD>

#include <cmath>
#include <numeric>
#include <string>
#include <iostream>
#include <vector>

#include "fblib/base/link_pragmas.h"

namespace fblib {
	namespace math{

		using Eigen::Map;

		typedef Eigen::NumTraits<double> EigenDoubleTraits;

		typedef Eigen::Matrix<double, 2, 2> Mat2;
		typedef Eigen::Matrix<double, 2, 3> Mat23;
		typedef Eigen::Matrix<double, 3, 3> Mat3;
		typedef Eigen::Matrix<double, 3, 4> Mat34;
		typedef Eigen::Matrix<double, 3, 5> Mat35;
		typedef Eigen::Matrix<double, 4, 1> Mat41;
		typedef Eigen::Matrix<double, 4, 3> Mat43;
		typedef Eigen::Matrix<double, 4, 4> Mat4;
		typedef Eigen::Matrix<double, 4, 6> Mat46;
		typedef Eigen::Matrix<float, 2, 2> Mat2f;
		typedef Eigen::Matrix<float, 2, 3> Mat23f;
		typedef Eigen::Matrix<float, 3, 3> Mat3f;
		typedef Eigen::Matrix<float, 3, 4> Mat34f;
		typedef Eigen::Matrix<float, 3, 5> Mat35f;
		typedef Eigen::Matrix<float, 4, 3> Mat43f;
		typedef Eigen::Matrix<float, 4, 4> Mat4f;
		typedef Eigen::Matrix<float, 4, 6> Mat46f;

		typedef Eigen::Vector2d Vec2;
		typedef Eigen::Vector3d Vec3;
		typedef Eigen::Vector4d Vec4;
		typedef Eigen::Vector2i Vec2i;
		typedef Eigen::Vector2f Vec2f;
		typedef Eigen::Vector3f Vec3f;
		typedef Eigen::Matrix<double, 5, 1>  Vec5;
		typedef Eigen::Matrix<double, 6, 1>  Vec6;
		typedef Eigen::Matrix<double, 7, 1>  Vec7;
		typedef Eigen::Matrix<double, 8, 1>  Vec8;
		typedef Eigen::Matrix<double, 9, 1>  Vec9;
		typedef Eigen::Matrix<double, 10, 1> Vec10;
		typedef Eigen::Matrix<double, 11, 1> Vec11;
		typedef Eigen::Matrix<double, 12, 1> Vec12;
		typedef Eigen::Matrix<double, 13, 1> Vec13;
		typedef Eigen::Matrix<double, 14, 1> Vec14;
		typedef Eigen::Matrix<double, 15, 1> Vec15;
		typedef Eigen::Matrix<double, 16, 1> Vec16;
		typedef Eigen::Matrix<double, 17, 1> Vec17;
		typedef Eigen::Matrix<double, 18, 1> Vec18;
		typedef Eigen::Matrix<double, 19, 1> Vec19;
		typedef Eigen::Matrix<double, 20, 1> Vec20;

		typedef Eigen::Quaternion<double> Quaternion;

		
		typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> Matu;

		typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> RMat3;

		//-- General purpose Matrix and Vector
		typedef Eigen::MatrixXd Mat;
		typedef Eigen::VectorXd Vec;
		typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> Vecu;
		typedef Eigen::MatrixXf Matf;
		typedef Eigen::VectorXf Vecf;

		typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;
		typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
		typedef Eigen::Matrix<double, 4, Eigen::Dynamic> Mat4X;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 2> MatX2;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatX3;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 4> MatX4;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 5> MatX5;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatX6;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 7> MatX7;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 8> MatX8;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 9> MatX9;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 15> MatX15;
		typedef Eigen::Matrix<double, Eigen::Dynamic, 16> MatX16;
		
		typedef Eigen::SparseMatrix<double> SparseMat;//!<稀疏矩阵，列优先
		typedef Eigen::SparseMatrix<double, Eigen::RowMajor> RSparseMat;//!<稀疏矩阵，行优先
		typedef Eigen::NumTraits<double> EigenDouble;
		/**
		 * \brief	返回一个数的平方
		 *
		 * \tparam	T	给定数的类型
		 * \param	x	给定的数
		 *
		 * \return	给定数的平方
		 */
		template<typename T>
		inline T Square(T x) {
			return x * x;
		}

		/// Clamp return the number if inside range, else min or max range.
		template<typename T>
		inline T clamp(const T & val, const T& min, const T & max)  {
			return std::max(min, std::min(val, max));
			//(val < min) ? val : ((val>max) ? val : max);
		}

		Mat3 CrossProductMatrix(const Vec3 &x);

		Mat3 RotationAroundX(double angle);

		Mat3 RotationAroundY(double angle);

		Mat3 RotationAroundZ(double angle);

		// Degree to Radian (suppose input in [0;360])
		inline double D2R(double degree)  {
			return degree* M_PI / 180.0;
		}

		// Radian to degree
		inline double R2D(double radian)  {
			return radian / M_PI * 180.0;
		}

		inline double SIGN(double x) {
			return x < 0.0 ? -1.0 : 1.0;
		}

		/**
		 * \brief	计算L1-范数，L1 norm = Sum (|x0| + |x1| + |xn|)
		 */
		template<typename TVec>
		inline double NormL1(const TVec &x) {
			return x.array().abs().sum();
		}

		/**
		 * \brief	计算L2-范数，L2 norm = Sqrt (Sum (x0^2 + x1^2 + xn^2))
		 */
		template<typename TVec>
		inline double NormL2(const TVec &x) {
			return x.norm();
		}

		/**
		 * \brief	计算L∞-范数，LInfinity norm = max (|x0|, |x1|, ..., |xn|)
		 */
		template<typename TVec>
		inline double NormLInfinity(const TVec &x) {
			return x.array().abs().maxCoeff();
		}

		/**
		 * \brief	计算曼哈顿距离对应L1-范数
		 */
		template<typename TVec>
		inline double DistanceL1(const TVec &x, const TVec &y) {
			return (x - y).array().abs().sum();
		}

		/**
		 * \brief	计算欧式距离（对应L2范数）
		 */
		template<typename TVec>
		inline double DistanceL2(const TVec &x, const TVec &y) {
			return (x - y).norm();
		}

		/**
		 * \brief	计算切比雪夫距离对应L∞范数
		 */
		template<typename TVec>
		inline double DistanceLInfinity(const TVec &x, const TVec &y) {
			return NormLInfinity(x - y);
		}

		/**
		 * \brief	通过L1-范数进行归一化处理
		 */
		template<typename TVec>
		inline double NormalizeL1(TVec *x) {
			double norm = NormL1(*x);
			*x /= norm;
			return norm;
		}

		/**
		* \brief	通过L2-范数进行归一化处理
		*/
		template<typename TVec>
		inline double NormalizeL2(TVec *x) {
			double norm = NormL2(*x);
			*x /= norm;
			return norm;
		}

		/**
		* \brief	通过L∞-范数进行归一化处理
		*/
		template<typename TVec>
		inline double NormalizeLInfinity(TVec *x) {
			double norm = NormLInfinity(*x);
			*x /= norm;
			return norm;
		}


		/**	\brief 通过SVD求解线性方程组Ax = 0；  
		 *	 当A的行数大于列数时，就需要求解最小二乘解，  
		 *   在||X||=1的约束下，其最小二乘解为矩阵A'A最小特征值所对应的特征向量
		 *   使用SVD分解矩阵A，[U S V] = svd(A); U 由 A*A'的特征向量组成，
		 *   V 由 A'*A的 特征向量组成，因此，奇异值矩阵S中最小的奇异值对应的V中的奇异向量即为最小二乘解。
		 */
		template <typename TMat, typename TVec>
		double Nullspace(TMat *A, TVec *nullspace) {
			if (A->rows() >= A->cols()) {
				Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
				(*nullspace) = svd.matrixV().col(A->cols() - 1);
				return svd.singularValues()(A->cols() - 1);
			}
			// Extend A with rows of zeros to make it square. It's a hack, but is
			// necessary until Eigen supports SVD with more columns than rows.
			TMat A_extended(A->cols(), A->cols());
			A_extended.block(A->rows(), 0, A->cols() - A->rows(), A->cols()).setZero();
			A_extended.block(0, 0, A->rows(), A->cols()) = (*A);
			return Nullspace(&A_extended, nullspace);
		}

		/// Solve the linear system Ax = 0 via SVD. Finds two solutions, x1 and x2, such
		/// that x1 is the best solution and x2 is the next best solution (in the L2
		/// norm sense). Store the solution in x1 and x2, such that ||x|| = 1.0. Return
		/// the singular value corresponding to the solution x1. Destroys A and resizes
		/// x if necessary.
		template <typename TMat, typename TVec1, typename TVec2>
		inline double Nullspace2(TMat *A, TVec1 *x1, TVec2 *x2) {
			if (A->rows() >= A->cols()) {
				Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
				TMat V = svd.matrixV();
				*x1 = V.col(A->cols() - 1);
				*x2 = V.col(A->cols() - 2);
				return svd.singularValues()(A->cols() - 1);
			}
			// Extend A with rows of zeros to make it square. It's a hack, but is
			// necessary until Eigen supports SVD with more columns than rows.
			TMat A_extended(A->cols(), A->cols());
			A_extended.block(A->rows(), 0, A->cols() - A->rows(), A->cols()).setZero();
			A_extended.block(0, 0, A->rows(), A->cols()) = (*A);
			return Nullspace2(&A_extended, x1, x2);
		}

		/**
		* \brief	基本的相机坐标系，认为参考点的方向向量为z轴
		*
		* \param	center	参考点的位置
		* \param	up	  	视点向上方向的向量.
		*
		* \return	世界坐标系转相机坐标系的矩阵
		*/
		Mat3 LookAt(const Vec3 &center, const Vec3 &up = Vec3::UnitY());

		/**
		* \brief	给出3参数的相机坐标系
		*
		* \param	eye_position   	视点的位置
		* \param	center_position	参考点的位置
		* \param	up_vector	   	视点向上方向的向量，通常为0.0, 1.0, 0.0
		*
		* \return	世界坐标系转相机坐标系的矩阵
		*/
		Mat3 LookAt2(const Vec3 &eye_position,
			const Vec3 &center_position = Vec3::Zero(),
			const Vec3 &up_vector = Vec3::UnitY());


#define SUM_OR_DYNAMIC(x,y) (x==Eigen::Dynamic||y==Eigen::Dynamic)?Eigen::Dynamic:(x+y)

		template<typename Derived1, typename Derived2>
		struct hstack_return {
			typedef typename Derived1::Scalar Scalar;
			enum {
				RowsAtCompileTime = Derived1::RowsAtCompileTime,
				ColsAtCompileTime = SUM_OR_DYNAMIC(Derived1::ColsAtCompileTime, Derived2::ColsAtCompileTime),
				Options = Derived1::Flags&Eigen::RowMajorBit ? Eigen::RowMajor : 0,
				MaxRowsAtCompileTime = Derived1::MaxRowsAtCompileTime,
				MaxColsAtCompileTime = SUM_OR_DYNAMIC(Derived1::MaxColsAtCompileTime, Derived2::MaxColsAtCompileTime)
			};
			typedef Eigen::Matrix < Scalar,
				RowsAtCompileTime,
				ColsAtCompileTime,
				Options,
				MaxRowsAtCompileTime,
				MaxColsAtCompileTime > type;
		};

		template<typename Derived1, typename Derived2>
		typename hstack_return<Derived1, Derived2>::type
			HStack(const Eigen::MatrixBase<Derived1>& lhs, const Eigen::MatrixBase<Derived2>& rhs) {
			typename hstack_return<Derived1, Derived2>::type res;
			res.resize(lhs.rows(), lhs.cols() + rhs.cols());
			res << lhs, rhs;
			return res;
		};


		template<typename Derived1, typename Derived2>
		struct vstack_return {
			typedef typename Derived1::Scalar Scalar;
			enum {
				RowsAtCompileTime = SUM_OR_DYNAMIC(Derived1::RowsAtCompileTime, Derived2::RowsAtCompileTime),
				ColsAtCompileTime = Derived1::ColsAtCompileTime,
				Options = Derived1::Flags&Eigen::RowMajorBit ? Eigen::RowMajor : 0,
				MaxRowsAtCompileTime = SUM_OR_DYNAMIC(Derived1::MaxRowsAtCompileTime, Derived2::MaxRowsAtCompileTime),
				MaxColsAtCompileTime = Derived1::MaxColsAtCompileTime
			};
			typedef Eigen::Matrix < Scalar,
				RowsAtCompileTime,
				ColsAtCompileTime,
				Options,
				MaxRowsAtCompileTime,
				MaxColsAtCompileTime > type;
		};

		template<typename Derived1, typename Derived2>
		typename vstack_return<Derived1, Derived2>::type
			VStack(const Eigen::MatrixBase<Derived1>& lhs, const Eigen::MatrixBase<Derived2>& rhs) {
			typename vstack_return<Derived1, Derived2>::type res;
			res.resize(lhs.rows() + rhs.rows(), lhs.cols());
			res << lhs, rhs;
			return res;
		};
#undef SUM_OR_DYNAMIC

		template<typename TMat>
		inline double FrobeniusNorm(const TMat &A) {
			return sqrt(A.array().abs2().sum());
		}

		template<typename TMat>
		inline double FrobeniusDistance(const TMat &A, const TMat &B) {
			return FrobeniusNorm(A - B);
		}


		/**
		 * \brief	抽取指定矩阵中的对应列
		 *
		 * \tparam	TMat 	矩阵类型
		 * \tparam	TCols	对应列类型
		 * \param	A	   	用来进行抽取列的矩阵
		 * \param	columns	指定要抽取的列数
		 *
		 * \return	抽取的矩阵
		 */
		template <typename TMat, typename TCols>
		TMat ExtractColumns(const TMat &A, const TCols &columns) {
			TMat compressed(A.rows(), columns.size());
			for (size_t i = 0; i < static_cast<size_t>(columns.size()); ++i) {
				compressed.col(i) = A.col(columns[i]);
			}
			return compressed;
		}

		/**
		* \brief	计算矩阵每行的均值和方差
		*
		* \param	A							待计算的矩阵
		* \param [in,out]	mean_pointer		均值指针
		* \param [in,out]	variance_pointer	方差指针
		*/
		void MeanAndVarianceAlongRows(const Mat &A,
			Vec *mean_pointer,
			Vec *variance_pointer);

		bool ExportMatToTextFile(const Mat &mat, const std::string &filename,
			const std::string &prefix = "A");

		inline int is_finite(const double val)
		{
#ifdef FBLIB_OS_WINDOWS
			return _finite(val);
#else
			return std::isfinite(val);
#endif
		}

		/**
		 * \brief	给出可迭代序列的最大值，最小值，平均值，和中值
		 *
		 * \tparam	Type			 	迭代序列类型
		 * \tparam	DataInputIterator	输入iterator类型.
		 * \param	begin		  	迭代器的begin
		 * \param	end			  	迭代器的end
		 * \param [in,out]	min   	输出迭代序列的最小值
		 * \param [in,out]	max   	输出迭代序列的最大值
		 * \param [in,out]	mean  	输出迭代序列的平均值
		 * \param [in,out]	median	输出迭代序列的中值
		 */
		template <typename Type, typename DataInputIterator>
		void MinMaxMeanMedian(DataInputIterator begin, DataInputIterator end,
			Type &min, Type &max, Type &mean, Type &median)
		{
			if (std::distance(begin, end) < 1)
				return;

			std::vector<Type> vec_val(begin, end);
			std::sort(vec_val.begin(), vec_val.end());
			min = vec_val[0];
			max = vec_val[vec_val.size() - 1];
			mean = std::accumulate(vec_val.begin(), vec_val.end(), 0.0)
				/ static_cast<double>(vec_val.size());
			median = vec_val[vec_val.size() / 2];
		}

		/**
		* \brief	控制台显示可迭代序列的最大值，最小值，平均值，和中值
		*
		* \tparam	Type			 	迭代序列类型
		* \tparam	DataInputIterator	输入iterator类型.
		* \param	begin		  	迭代器的begin
		* \param	end			  	迭代器的end
		 */
		template <typename Type, typename DataInputIterator>
		void MinMaxMeanMedian(DataInputIterator begin, DataInputIterator end)
		{
			Type min, max, mean, median;
			MinMaxMeanMedian(begin, end, min, max, mean, median);
			std::cout << "\n"
				<< "\t min: " << min << "\n"
				<< "\t mean: " << mean << "\n"
				<< "\t median: " << median << std::endl
				<< "\t max: " << max << std::endl;
		}
	} // namespace math
} // namespace fblib

#endif // FBLIB_MATH_NUMERIC_H_
