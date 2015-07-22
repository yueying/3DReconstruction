#ifndef FBLIB_FEATURE_METRIC_H_
#define FBLIB_FEATURE_METRIC_H_

namespace fblib {
	namespace feature {
		//TODO:这边可以采用SSE，后续添加
		template<typename T>
		struct Accumulator { typedef T Type; };
		template<>
		struct Accumulator < unsigned char > { typedef float Type; };
		template<>
		struct Accumulator < unsigned short > { typedef float Type; };
		template<>
		struct Accumulator < unsigned int > { typedef float Type; };
		template<>
		struct Accumulator < char > { typedef float Type; };
		template<>
		struct Accumulator < short > { typedef float Type; };
		template<>
		struct Accumulator < int > { typedef float Type; };

		/**
		 * \brief	简单的计算欧式距离的平方
		 *
		 * \tparam	T	参数类型
		 */
		template<class T>
		struct SquaredEuclideanDistanceSimple
		{
			typedef T ElementType;
			typedef typename Accumulator<T>::Type ResultType;

			template <typename Iterator1, typename Iterator2>
			ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const
			{
				ResultType result = ResultType();
				ResultType diff;
				for (size_t i = 0; i < size; ++i) {
					diff = *a++ - *b++;
					result += diff*diff;
				}
				return result;
			}
		};

		/**
		* \brief	计算欧式距离的平方，向量化版本
		*
		* \tparam	T	参数类型
		*/
		template<class T>
		struct SquaredEuclideanDistanceVectorized
		{
			typedef T ElementType;
			typedef typename Accumulator<T>::Type ResultType;

			template <typename Iterator1, typename Iterator2>
			ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const
			{
				ResultType result = ResultType();
				ResultType diff0, diff1, diff2, diff3;
				Iterator1 last = a + size;
				Iterator1 lastgroup = last - 3;

				// 每次循环处理4项，用于提高效率
				while (a < lastgroup) {
					diff0 = a[0] - b[0];
					diff1 = a[1] - b[1];
					diff2 = a[2] - b[2];
					diff3 = a[3] - b[3];
					result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
					a += 4;
					b += 4;
				}
				// 对于非标准向量长度，用于处理最后的0-3个像素
				while (a < last) {
					diff0 = *a++ - *b++;
					result += diff0 * diff0;
				}
				return result;
			}
		};

	}  // namespace feature
}  // namespace fblib


#endif // FBLIB_FEATURE_METRIC_H_
