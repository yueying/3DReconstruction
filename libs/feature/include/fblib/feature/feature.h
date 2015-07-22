#ifndef FBLIB_FEATURE_FEATURE_H_
#define FBLIB_FEATURE_FEATURE_H_

#include <iostream>
#include <iterator>
#include <fstream>
#include <string>
#include <vector>

#include "fblib/math/numeric.h"
using namespace fblib::math;

namespace fblib {
	namespace feature{
		/**
		 * 特征的抽象类
		 */
		class FeatureBase {
		public:
			virtual inline ~FeatureBase() {};

			virtual std::ostream& print(std::ostream& output) const = 0;
			virtual std::istream& read(std::istream& input) = 0;
		};

		/**	重载<<操作符，进行输出
		 */
		inline std::ostream& operator<<(std::ostream& out, const FeatureBase& obj)
		{
			return obj.print(out); 
		}

		inline std::istream& operator>>(std::istream& in, FeatureBase& obj)
		{
			return obj.read(in);
		}

		/**
		 * 点特征的基类，用于存储特征点的位置
		 */
		class PointFeature : public FeatureBase {
		public:
			virtual inline ~PointFeature() {};

			inline PointFeature(float x = 0.0f, float y = 0.0f)
				: _coords(x, y) {}

			inline float x() const { return _coords(0); }
			inline float y() const { return _coords(1); }
			

			inline float& x() { return _coords(0); }
			inline float& y() { return _coords(1); }
			/**	得到特征点的坐标，二维向量
			*/
			inline Vec2f coords() const { return _coords; }
			/**	得到特征点的坐标，二维向量
			 */
			inline Vec2f& coords() { return _coords; }

			virtual inline std::ostream& print(std::ostream& os) const
			{
				return os << _coords(0) << " " << _coords(1);
			}

			virtual inline std::istream& read(std::istream& in)
			{
				return in >> _coords(0) >> _coords(1);
			}

		protected:
			Vec2f _coords;  // (x, y).
		};

		/**
		 * 尺度不变，方向不变点特征的基类
		 * 在基类点特征中添加尺度和方向
		 */
		class ScalePointFeature : public PointFeature {
		public:
			virtual ~ScalePointFeature() {};

			ScalePointFeature(float x = 0.0f, float y = 0.0f,
				float scale = 0.0f, float orient = 0.0f)
				: PointFeature(x, y)
				, _scale(scale)
				, _orientation(orient) {}

			inline float scale() const { return _scale; }
			inline float& scale() { return _scale; }
			inline float orientation() const { return _orientation; }
			inline float& orientation() { return _orientation; }

			bool operator ==(const ScalePointFeature& b) const {
				return (_scale == b.scale()) &&
					(_orientation == b.orientation()) &&
					(x() == b.x()) && (y() == b.y());
			};

			virtual std::ostream& print(std::ostream& os) const
			{
				return PointFeature::print(os) << " " << _scale << " " << _orientation;
			}

			virtual std::istream& read(std::istream& in)
			{
				return PointFeature::read(in) >> _scale >> _orientation;
			}

		protected:
			float _scale;        // 单位像素
			float _orientation;  // 单位弧度
		};

		/**	从文件中读取特征
		 */
		template<typename FeaturesT >
		static bool LoadFeatsFromFile(
			const std::string & feature_filename,
			FeaturesT & vec_feat)
		{
			vec_feat.clear();
			bool is_ok = false;

			std::ifstream file_in(feature_filename.c_str());
			std::copy(
				std::istream_iterator<typename FeaturesT::value_type >(file_in),
				std::istream_iterator<typename FeaturesT::value_type >(),
				std::back_inserter(vec_feat));
			is_ok = !file_in.bad();
			file_in.close();
			return is_ok;
		}

		/**	将特征写入到文件中
		 */
		template<typename FeaturesT >
		static bool saveFeatsToFile(
			const std::string & feature_filename,
			FeaturesT & vec_feat)
		{
			std::ofstream file(feature_filename.c_str());
			std::copy(vec_feat.begin(), vec_feat.end(),
				std::ostream_iterator<typename FeaturesT::value_type >(file, "\n"));
			bool is_ok = file.good();
			file.close();
			return is_ok;
		}

		/**	将点特征转矩阵的形式
		 */
		template< typename FeaturesT, typename MatT >
		void PointsToMat(
			const FeaturesT & vec_feats,
			MatT & m)
		{
			m.resize(2, vec_feats.size());
			typedef typename FeaturesT::value_type ValueT; // 容器类型
			typedef typename MatT::Scalar Scalar; // 输出矩阵的类型

			size_t i = 0;
			for (typename FeaturesT::const_iterator iter = vec_feats.begin();
				iter != vec_feats.end(); ++iter, ++i)
			{
				const ValueT & feat = *iter;
				m.col(i)(0) = Scalar(feat.x());
				m.col(i)(1) = Scalar(feat.y());
			}
		}
	} //namespace feature
} // namespace fblib

#endif // FBLIB_FEATURE_FEATURE_H_
