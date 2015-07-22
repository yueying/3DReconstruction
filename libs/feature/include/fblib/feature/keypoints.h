#ifndef FBLIB_FEATURE_KEYPOINTS_H_
#define FBLIB_FEATURE_KEYPOINTS_H_

#include "fblib/feature/feature.h"
#include "fblib/feature/descriptor.h"
#include <string>

namespace fblib {
	namespace feature{

		/**
		 * \brief	对关键点的特征及描述子进行保存和读取操作
		 * 			typedef vector<ScalePointFeature> featsT;
		 *          typedef vector<Descriptor<uchar,128> > descsT;
		 *          KeypointSet< featsT, descsT > keypoint_set;
		 *
		 * \tparam	FeaturesT   	Type of the features t.
		 * \tparam	DescriptorsT	Type of the descriptors t.
		 */
		template<typename FeaturesT, typename DescriptorsT>
		class KeypointSet {
		public:
			// 存放特征和描述子的别名
			typedef typename FeaturesT::value_type FeatureT;
			typedef typename DescriptorsT::value_type DescriptorT;

			/**
			 * \brief	从文件中读取特征及对应的描述子
			 *
			 * \param	feature_filename	特征文件的文件名
			 * \param	descs_filename  	描述子文件的文件名
			 *
			 * \return	true if it succeeds, false if it fails.
			 */			
			bool loadFromFile(
				const std::string &feature_filename,
				const std::string &descs_filename)
			{
				return LoadFeatsFromFile(feature_filename, _feats)
					& LoadDescsFromFile(descs_filename, _descs);
			}

			/**
			* \brief	单独将特征和描述子导出
			*
			* \param	feature_filename	特征文件的文件名
			* \param	descs_filename  	描述子文件的文件名
			*
			* \return	true if it succeeds, false if it fails.
			*/
			bool saveToFile(
				const std::string& feature_filename,
				const std::string& descs_filename) const
			{
				return saveFeatsToFile(feature_filename, _feats)
					& SaveDescsToFile(descs_filename, _descs);
			}

			/**
			* \brief	从文件中读取特征及对应的描述子，描述子为二进制形式
			*
			* \param	feature_filename	特征文件的文件名
			* \param	descs_filename  	描述子文件的文件名
			*
			* \return	true if it succeeds, false if it fails.
			*/
			bool loadFromBinFile(
				const std::string& feature_filename,
				const std::string& descs_filename)
			{
				return LoadFeatsFromFile(feature_filename, _feats)
					& LoadDescsFromBinFile(descs_filename, _descs);
			}

			/**
			 * \brief	单独将特征和描述子导出，描述子导出为二进制形式
			 *
			 * \param	feature_filename	特征文件的文件名
			 * \param	descs_filename  	描述子文件的文件名
			 *
			 * \return	true if it succeeds, false if it fails.
			 */			
			bool saveToBinFile(
				const std::string& feature_filename,
				const std::string& descs_filename) const
			{
				return saveFeatsToFile(feature_filename, _feats)
					& SaveDescsToBinFile(descs_filename, _descs);
			}

			/**	获取特征
			 */
			FeaturesT & features() { return _feats; }
			const FeaturesT & features() const { return _feats; }

			/**	获取描述子
			 */
			DescriptorsT & descriptors() { return _descs; }
			const DescriptorsT & descriptors() const { return _descs; }

		private:
			FeaturesT _feats;
			DescriptorsT _descs;
		};
	} //namespace feature
} // namespace fblib

#endif // FBLIB_FEATURE_KEYPOINTS_H_
