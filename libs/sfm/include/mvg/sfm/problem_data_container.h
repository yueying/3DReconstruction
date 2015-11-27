#ifndef MVG_SFM_PROBLEM_DATA_CONTAINER_H
#define MVG_SFM_PROBLEM_DATA_CONTAINER_H

#include <vector>

namespace mvg{
	namespace sfm{

		/**
		 * \brief	Bundle Adjustment数据集容器
		 *          可以对每个相机进行单独设置，默认使用7个参数
		 *          (Rotation(angle,axis), t, focal)
		 *
		 * \tparam	NCamParam	相机参数个数
		 */
		template<unsigned char NCamParam = 7>
		class BAProblemData {
		public:

			// 相机参数的数目
			static const unsigned char kCamParam = NCamParam;
			// 得到观测到的3d点的数目
			size_t num_observations()    const { return num_observations_; }
			// 返回一个指向观测点[X_0, ... ,X_n]的指针
			const double* observations() const { return &observations_[0]; }
			// 返回指向相机数据的指针
			double* mutable_cameras() {
				return &parameters_[0];
			}
			// 返回指向3d点数据指针
			double* mutable_points()  {
				return &parameters_[0] + NCamParam * num_cameras_;
			}

			// 返回一个指针指向观测i个观察项的相机
			double* mutable_camera_for_observation(size_t i) {
				return mutable_cameras() + camera_index_[i] * NCamParam;
			}
			// 返回针对i观察项所指向的3d点的指针
			double* mutable_point_for_observation(size_t i) {
				return mutable_points() + point_index_[i] * 3;
			}

			size_t num_cameras_;      //!< 相机的个数
			size_t num_points_;       //!< 三维观测点的个数
			size_t num_observations_; //!< 对应所有观测点数，相机个数*三维点数
			size_t num_parameters_;   //!< 参数的个数为 ( NCamParam * #Cam + 3 * #Points)

			std::vector<size_t> point_index_;  // 投影对应的2d点的索引
			std::vector<size_t> camera_index_; // 投影对应的相机的索引
			std::vector<double> observations_; // 3D观测点

			std::vector<double> parameters_;  // 相机参数
		};

		/**
		 * \brief BA的数据容器，相机参数同内外参进行传递	External parameter => 6: [Rotation(angle,axis), t]
		 *        Intrinsic => 1: [Focal]
		 *
		 * \tparam	NExternalParam 	外参的参数个数
		 * \tparam	NIntrinsicParam	内参的参数个数
		 */
		template<
			unsigned char NExternalParam = 6,
			unsigned char NIntrinsicParam = 1>
		class BA_Problem_data_camMotionAndIntrinsic {
		public:

			/// 相机内外参的个数
			static const unsigned char kExternalParam = NExternalParam;
			static const unsigned char kIntrinsicParam = NIntrinsicParam;

			/// 返回观测到三维点的个数
			size_t num_observations()    const { return num_observations_; }
			/// 返回一个指针指向观测点数据[X_0, ... ,X_n]
			const double* observations() const { return &observations_[0]; }
			/// 返回一个指针指向相机外参数据
			double* mutable_cameras_extrinsic() {
				return &parameters_[0];
			}
			/// 返回一个指针相机内参数据
			double* mutable_cameras_intrinsic() {
				return &parameters_[0] + NExternalParam * num_cameras_;
			}
			/// 返回一个指针指向点数据
			double* mutable_points()  {
				return &parameters_[0]
					+ NExternalParam * num_cameras_
					+ NIntrinsicParam * num_intrinsic_;
			}

			/// 返回针对i观察项所指向的相机外参的指针
			double* mutable_camera_extrinsic_for_observation(size_t i) {
				return mutable_cameras_extrinsic() + camera_index_extrinsic[i] * NExternalParam;
			}
			/// 返回针对i观察项所指向的相机内参的指针
			double* mutable_camera_intrisic_for_observation(size_t i) {
				return mutable_cameras_intrinsic() + camera_index_intrinsic[i] * NIntrinsicParam;
			}

			/// 返回针对i观察项所指向的3d点的指针
			double* mutable_point_for_observation(size_t i) {
				return mutable_points() + point_index_[i] * 3;
			}

			size_t num_cameras_;      //!< 相机的个数
			size_t num_intrinsic_;    //!< 对应相机内参个数
			size_t num_points_;       //!< 三维点个数
			size_t num_observations_; //!< 观察点个数 num_cameras_*num_points_
			// 参数个数: NIntrinsicParam * num_cameras_ + NIntrinsicParam * num_intrinsic_ + 3 * num_points_
			size_t num_parameters_;

			std::vector<size_t> point_index_;  // 反投影对应的2d点的索引
			std::vector<size_t> camera_index_extrinsic; // 反投影对应的相机外参索引
			std::vector<size_t> camera_index_intrinsic; // 反投影对应的相机内参索引
			std::vector<double> observations_; // 3D观察点

			// 相机所有参数([R|t]_0,...,[R|t]actual_camera_num_,[f]_0,...,[f]actual_camera_num_)
			std::vector<double> parameters_;

		};

	} // namespace sfm
} // namespace mvg

#endif // MVG_SFM_PROBLEM_DATA_CONTAINER_H
