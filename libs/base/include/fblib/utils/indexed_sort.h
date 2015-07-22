#ifndef FBLIB_UTILS_INDEXED_SORT_H_
#define FBLIB_UTILS_INDEXED_SORT_H_

namespace fblib{
	namespace utils{

		/**
		 * \brief	从小到大排序封装的packet
		 *
		 * \tparam	T1	元素的类型
		 * \tparam	T2	索引的类型
		 */
		template<typename T1, typename T2>
		struct SortIndexPacketAscend {
			T1 val;
			T2 index;
		};

		/**
		* \brief	从大到小排序封装的packet
		*
		* \tparam	T1	元素的类型
		* \tparam	T2	索引的类型
		*/
		template<typename T1, typename T2>
		struct SortIndexPacketDescend  {
			T1 val;
			T2 index;
		};

		template<typename T1, typename T2>
		inline bool operator< (const SortIndexPacketAscend<T1, T2>& A,
			const SortIndexPacketAscend<T1, T2>& B) {
			return A.val < B.val;
		}

		template<typename T1, typename T2>
		inline bool operator< (const SortIndexPacketDescend<T1, T2>& A,
			const SortIndexPacketDescend<T1, T2>& B)  {
			return A.val > B.val;
		}

		/**
		 * \brief	默认对所有元素进行排序，或者仅或者最小的n个元素
		 *
		 * \tparam	packet_type	排序封装元素类型，包含索引及元素
		 * \tparam	eT		   	元素类型.
		 * \param [in,out]	packet_vec	输入待排序类型，包含索引及元素.
		 * \param	in_mem			  	内存中的数值
		 * \param	n_smallest_element	最小元素数目.
		 */
		template<typename packet_type, typename eT>
		void inline SortIndexHelper(std::vector<packet_type>& packet_vec,
			const eT* in_mem, int n_smallest_element = -1)  {
			const size_t n_elem = packet_vec.size();
			//构建packet
			for (size_t i = 0; i < n_elem; ++i)  {
				packet_vec[i].val = in_mem[i];
				packet_vec[i].index = i;
			}
			if (n_smallest_element == -1)
				std::sort(packet_vec.begin(), packet_vec.end());
			else
				std::partial_sort(packet_vec.begin(), packet_vec.begin() + n_smallest_element,
				packet_vec.end());
		}
	} // namespace utils
} // namespace fblib


#endif // FBLIB_UTILS_INDEXED_SORT_H_
