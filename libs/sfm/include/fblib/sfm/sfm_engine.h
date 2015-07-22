#ifndef FBLIB_SFM_SFM_ENGINE_H
#define FBLIB_SFM_SFM_ENGINE_H

#include <string>

namespace fblib{
	namespace sfm{

		
		/** \brief	重建引擎的基类 */
		class ReconstructionEngine
		{
		public:

			/**
			 * \brief	构造函数.
			 *
			 * \param	image_path  	图像路径
			 * \param	matches_path	对应图像匹配路径
			 * \param	out_dir			处理输出目录
			 */
			ReconstructionEngine(const std::string &image_path,
				const std::string &matches_path,
				const std::string &out_dir)
				:image_path_(image_path),
				matches_path_(matches_path),
				out_dir_(out_dir)
			{
			}

			virtual bool Process() = 0;

		protected:
			std::string image_path_;    //!< 图像集的路径
			std::string matches_path_;  //!< 图像集对应的特征
			std::string out_dir_; //!< 输出目录
		};
	}// namespace sfm
} // namespace fblib

#endif // FBLIB_SFM_SFM_ENGINE_H
