#ifndef MVG_FEATURE_DESCRIPTOR_H_
#define MVG_FEATURE_DESCRIPTOR_H_

#include <iostream>
#include <iterator>
#include <fstream>
#include <string>
#include <vector>

#include "mvg/math/numeric.h"

namespace mvg {
	namespace feature{

		/**
		 * \brief	特征描述子(含有N个类型T的数组）
		 * SiftDescriptor => <uchar,128> 或 <float,128>
		 * SurfDescriptor => <float,64>
		 *
		 * \tparam	T	描述子元素类型
		 * \tparam	N	非类型模板参数，指定描述子维度
		 */
		template <typename T, std::size_t N>
		class Descriptor
		{
		public:
			typedef T bin_type;
			typedef std::size_t size_type;
	
			static const size_type kStaticSize = N;//!< 编译期间确定描述子的维度

			/**	构造函数
			 */
			inline Descriptor() {}

			/**	返回描述子维度
			 */
			inline size_type size() const { return N; }

			/**	重载操作，以数组的方式进行获取
			 */
			inline bin_type &operator[](std::size_t i) { return data[i]; }
			inline bin_type operator[](std::size_t i) const { return data[i]; }

			/**	得到特征描述子
			 */
			inline bin_type *getData() const { return (bin_type*)(&data[0]); }

			/**	输出流
			 */
			std::ostream& print(std::ostream& os) const;
			/**	输入流
			 */
			std::istream& read(std::istream& in);

		private:
			bin_type data[N];
		};

		/**	重载操作符<<，更简单的进行输出
		 */
		template <typename T, std::size_t N>
		inline std::ostream &operator<<(std::ostream &out, const Descriptor<T, N> &obj)
		{
			return obj.print(out);
		}

		/**	重载操作法>>，更简单的进行输入
		 */
		template <typename T, std::size_t N>
		inline std::istream& operator>>(std::istream& in, Descriptor<T, N>& obj)
		{
			return obj.read(in);
		}

		template<typename T>
		inline std::ostream& printT(std::ostream& os, T *tab, size_t N)
		{
			std::copy(tab, &tab[N], std::ostream_iterator<T>(os, " "));
			return os;
		}

		/**	对unsigned char类型的输出进行特别处理
		 */
		template<>
		inline std::ostream& printT<unsigned char>(std::ostream& os, unsigned char *tab, size_t N)
		{
			for (size_t i = 0; i < N; ++i)
				os << static_cast<int>(tab[i]) << " ";
			return os;
		}

		template<typename T>
		inline std::istream& readT(std::istream &is, T *tab, size_t N)
		{
			for (size_t i = 0; i < N; ++i) is >> tab[i];
			return is;
		}

		template<>
		inline std::istream& readT<unsigned char>(std::istream& is, unsigned char *tab, size_t N)
		{
			int temp = -1;
			for (size_t i = 0; i < N; ++i){
				is >> temp; tab[i] = (unsigned char)temp;
			}
			return is;
		}

		template<typename T, std::size_t N>
		std::ostream& Descriptor<T, N>::print(std::ostream& os) const
		{
			return printT<T>(os, (T*)&data[0], N);
		}

		template<typename T, std::size_t N>
		std::istream& Descriptor<T, N>::read(std::istream& in)
		{
			return readT<T>(in, (T*)&data[0], N);
		}

		/**	从文件中读取特征描述
		 */
		template<typename DescriptorsT >
		static bool LoadDescsFromFile(
			const std::string & descs_filename,
			DescriptorsT & vec_desc)
		{
			vec_desc.clear();

			std::ifstream file_in(descs_filename.c_str());
			// 定义istream_iterator时不为它指定istream对象，它即代表了end-of-file(文件尾)。
			std::copy(
				std::istream_iterator<typename DescriptorsT::value_type >(file_in),
				std::istream_iterator<typename DescriptorsT::value_type >(),
				std::back_inserter(vec_desc));
			bool is_ok = !file_in.bad();
			file_in.close();
			return is_ok;
		}

		/**	将描述子内容写入文件中
		 */
		template<typename DescriptorsT >
		static bool SaveDescsToFile(
			const std::string & descs_filename,
			DescriptorsT & vec_desc)
		{
			std::ofstream file(descs_filename.c_str());
			std::copy(vec_desc.begin(), vec_desc.end(),
				std::ostream_iterator<typename DescriptorsT::value_type >(file, "\n"));
			bool is_ok = file.good();
			file.close();
			return is_ok;
		}


		/**	以二进制的形式从文件中读取描述子
		 */
		template<typename DescriptorsT >
		static bool LoadDescsFromBinFile(
			const std::string &descs_filename,
			DescriptorsT &vec_desc)
		{
			typedef typename DescriptorsT::value_type DescValue;

			vec_desc.clear();
			std::ifstream file_in(descs_filename.c_str(), std::ios::in | std::ios::binary);

			std::size_t card_desc = 0;
			file_in.read((char*)&card_desc, sizeof(std::size_t));
			vec_desc.resize(card_desc);
			for (typename DescriptorsT::const_iterator iter = vec_desc.begin();
				iter != vec_desc.end(); ++iter) {
				file_in.read((char*)(*iter).getData(),
					DescValue::kStaticSize*sizeof(typename DescValue::bin_type));
			}
			bool is_ok = !file_in.bad();
			file_in.close();
			return is_ok;
		}

		/**	将特征描述以二进制的形式写入到文件中
		 */
		template<typename DescriptorsT >
		static bool SaveDescsToBinFile(
			const std::string & descs_filename,
			DescriptorsT & vec_desc)
		{
			typedef typename DescriptorsT::value_type DescValue;
			std::ofstream file(descs_filename.c_str(), std::ios::out | std::ios::binary);
			//写入描述子的数目
			const std::size_t card_desc = vec_desc.size();
			file.write((const char*)&card_desc, sizeof(std::size_t));
			for (typename DescriptorsT::const_iterator iter = vec_desc.begin();
				iter != vec_desc.end(); ++iter) {
				file.write((const char*)(*iter).getData(),
					DescValue::kStaticSize*sizeof(typename DescValue::bin_type));
			}
			bool is_ok = file.good();
			file.close();
			return is_ok;
		}
	} //namespace feature
}  // namespace mvg

#endif // MVG_FEATURE_DESCRIPTOR_H_
