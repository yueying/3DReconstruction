#include "base_precomp.h"
#include "fblib/math/numeric.h"

#include <iostream>
#include <fstream>
#include <string>

namespace fblib {
	namespace math{
		/**	定义反对称矩阵，用于叉乘计算
		 */
		Mat3 CrossProductMatrix(const Vec3 &x) {
			Mat3 X;
			X << 0, -x(2), x(1),
				x(2), 0, -x(0),
				-x(1), x(0), 0;
			return X;
		}

		/**	绕X轴旋转度数转矩阵，即滚动角，单位弧度
		 */
		Mat3 RotationAroundX(double angle) {
			return Eigen::AngleAxisd(angle, Vec3::UnitX()).toRotationMatrix();
		}
		/**	绕Y轴旋转度数转矩阵，即俯仰角，单位弧度
		*/
		Mat3 RotationAroundY(double angle) {
			return Eigen::AngleAxisd(angle, Vec3::UnitY()).toRotationMatrix();
		}
		/**	绕Z轴旋转度数转矩阵，即方位角，单位弧度
		*/
		Mat3 RotationAroundZ(double angle) {
			return Eigen::AngleAxisd(angle, Vec3::UnitZ()).toRotationMatrix();
		}

		/**
		 * \brief	基本的相机坐标系，认为参考点的方向向量为z轴
		 *
		 * \param	center	参考点的位置
		 * \param	up	  	视点向上方向的向量.
		 *
		 * \return	世界坐标系转相机坐标系的矩阵
		 */
		Mat3 LookAt(const Vec3 &center, const Vec3 &up) {
			Vec3 zc = center.normalized();//向量n
			Vec3 xc = up.cross(zc).normalized();//向量u
			Vec3 yc = zc.cross(xc);//向量v
			Mat3 R;
			R.row(0) = xc;
			R.row(1) = yc;
			R.row(2) = zc;
			return R;
		}

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
			const Vec3 &center_position,
			const Vec3 &up_vector)
		{
			Vec3 forward, side, up;
			Mat3 matrix2, resultMatrix;
			//------------------
			forward = center_position - eye_position;
			forward.normalize();
			//------------------
			//Side = forward x up
			//ComputeNormalOfPlane(side, forward, up_vector);
			side[0] = (forward[1] * up_vector[2]) - (forward[2] * up_vector[1]);
			side[1] = (forward[2] * up_vector[0]) - (forward[0] * up_vector[2]);
			side[2] = (forward[0] * up_vector[1]) - (forward[1] * up_vector[0]);
			side.normalize();
			//------------------
			//Recompute up as: up = side x forward
			//ComputeNormalOfPlane(up, side, forward);
			up[0] = (side[1] * forward[2]) - (side[2] * forward[1]);
			up[1] = (side[2] * forward[0]) - (side[0] * forward[2]);
			up[2] = (side[0] * forward[1]) - (side[1] * forward[0]);

			//------------------
			matrix2(0) = side[0];
			matrix2(1) = side[1];
			matrix2(2) = side[2];
			//------------------
			matrix2(3) = up[0];
			matrix2(4) = up[1];
			matrix2(5) = up[2];
			//------------------
			matrix2(6) = -forward[0];
			matrix2(7) = -forward[1];
			matrix2(8) = -forward[2];

			return matrix2;
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
			Vec *variance_pointer) {

			Vec &mean = *mean_pointer;
			Vec &variance = *variance_pointer;
			Mat::Index n = A.rows();
			double m = static_cast<double>(A.cols());
			mean = variance = Vec::Zero(n);

			for (Mat::Index i = 0; i < n; ++i) {
				mean(i) += A.row(i).array().sum();
				variance(i) += (A.row(i).array() * A.row(i).array()).array().sum();
			}

			mean /= m;
			for (Mat::Index i = 0; i < n; ++i) {
				variance(i) = variance(i) / m - Square(mean(i));
			}
		}

		bool ExportMatToTextFile(const Mat & mat, const std::string & filename,
			const std::string & prefix)
		{
			bool is_ok = false;
			std::ofstream outfile;
			outfile.open(filename.c_str(), std::ios_base::out);
			if (outfile.is_open()) {
				outfile << prefix << "=[" << std::endl;
				for (int j = 0; j < mat.rows(); ++j)  {
					for (int i = 0; i < mat.cols(); ++i)  {
						outfile << mat(j, i) << " ";
					}
					outfile << ";\n";
				}
				outfile << "];";
				is_ok = true;
			}
			outfile.close();
			return is_ok;
		}
	}
}  // namespace fblib