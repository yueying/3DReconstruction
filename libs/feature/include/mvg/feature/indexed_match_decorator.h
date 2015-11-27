#ifndef MVG_FEATURE_INDEXED_MATCH_DECORATOR_H_
#define MVG_FEATURE_INDEXED_MATCH_DECORATOR_H_

#include <iostream>
#include "mvg/feature/indexed_match.h"
#include "mvg/feature/features.h"

namespace mvg {
	namespace feature {

		/**
		 * \brief	IndexedMatch 装饰类，用于对x，y坐标进行排序
		 *
		 * \tparam	T	参数类型
		 */
		template<class T = float>
		class IndexedMatchDecorator
		{
			struct IndexedMatchDecoratorStruct
			{
				IndexedMatchDecoratorStruct(
					T xa, T ya,
					T xb, T yb,
					const IndexedMatch & ind) {

					x1 = xa; y1 = ya;
					x2 = xb; y2 = yb;
					index = ind;
				}

				/**	按字典顺序进行排序，用于删除重复值
				 */
				friend bool operator<(const IndexedMatchDecoratorStruct& m1,
					const IndexedMatchDecoratorStruct& m2)  {

					if (m1 == m2) return false;

					if (m1.x1 < m2.x1)
						return m1.y1 < m2.y1;
					else
						if (m1.x1 > m2.x1)
							return m1.y1 < m2.y1;
					return m1.x1 < m2.x1;
				}

				/**	比较操作符重载
				 */
				friend bool operator==(const IndexedMatchDecoratorStruct& m1,
					const IndexedMatchDecoratorStruct& m2)  {

					return (m1.x1 == m2.x1 && m1.y1 == m2.y1 &&
						m1.x2 == m2.x2 && m1.y2 == m2.y2);
				}

				T x1, y1, x2, y2;
				IndexedMatch index;
			};

		public:
			IndexedMatchDecorator(const std::vector<IndexedMatch> & vec_matches,
				const std::vector<ScalePointFeature> & left_feature,
				const std::vector<ScalePointFeature> & right_feature)
				:vec_matches_(vec_matches)
			{
				for (size_t i = 0; i < vec_matches.size(); ++i) {
					const size_t I = vec_matches[i]._i;
					const size_t J = vec_matches[i]._j;
					vec_decored_matches_.push_back(
						IndexedMatchDecoratorStruct(left_feature[I].x(), left_feature[I].y(),
						right_feature[J].x(), right_feature[J].y(), vec_matches[i]));
				}
			}

			IndexedMatchDecorator(const std::vector<IndexedMatch> & vec_matches,
				const Mat & left_feature,
				const Mat & right_feature)
				:vec_matches_(vec_matches)
			{
				for (size_t i = 0; i < vec_matches.size(); ++i) {
					const size_t I = vec_matches[i]._i;
					const size_t J = vec_matches[i]._j;
					vec_decored_matches_.push_back(
						IndexedMatchDecoratorStruct(left_feature.col(I)(0), left_feature.col(I)(1),
						right_feature.col(J)(0), right_feature.col(J)(1), vec_matches[i]));
				}
			}

			/**	删除重复项，相同的(x1,y1) 出现多次
			 */
			size_t getDeduplicated(std::vector<IndexedMatch> & vec_matches)
			{
				size_t sizeBefore = vec_decored_matches_.size();
				std::set<IndexedMatchDecoratorStruct> set_deduplicated(
					vec_decored_matches_.begin(), vec_decored_matches_.end());
				vec_decored_matches_.assign(set_deduplicated.begin(), set_deduplicated.end());

				vec_matches.resize(vec_decored_matches_.size());
				for (size_t i = 0; i < vec_decored_matches_.size(); ++i)  {
					const IndexedMatch & idx_match = vec_decored_matches_[i].index;
					vec_matches[i] = idx_match;
				}

				return sizeBefore != vec_matches.size();
			}

			/**
			  * 保存匹配项
			  * \param file_name  保存匹配项的文件名
			  * \return bool True if everything was ok, otherwise false.
			  */
			bool saveMatch(const char* file_name) const  {
				std::ofstream f(file_name);
				if (f.is_open()) {
					std::copy(vec_decored_matches_.begin(), vec_decored_matches_.end(),
						std::ostream_iterator<IndexedMatchDecoratorStruct>(f, ""));
				}
				return f.is_open();
			}

			friend std::ostream& operator<<(std::ostream &os, const IndexedMatchDecoratorStruct &m)
			{
				return os << m.x1 << " " << m.y1 << " " << m.x2 << " " << m.y2 << "\n";
			}

		private:
			std::vector<IndexedMatch> vec_matches_;
			std::vector<IndexedMatchDecoratorStruct> vec_decored_matches_;
		};

	}  // namespace feature
}  // namespace mvg

#endif // MVG_FEATURE_INDEXED_MATCH_DECORATOR_H_
