#ifndef FBLIB_IMAGE_TRAITS_H_
#define FBLIB_IMAGE_TRAITS_H_


#define IMAGE_DEPTH_1U     1
#define IMAGE_DEPTH_8U     8
#define IMAGE_DEPTH_16U   16
#define IMAGE_DEPTH_32F   32

namespace fblib
{
	namespace image
	{

		template<typename _Tp> class DataType
		{
		public:
			typedef _Tp         value_type;
			typedef value_type  work_type;
			typedef value_type  channel_type;
			enum {
				generic_type = 1,
				depth = -1,
				channels = 1
			};
		};

		template<> class DataType < bool >
		{
		public:
			typedef bool        value_type;
			typedef value_type  channel_type;
			enum {
				depth = IMAGE_DEPTH_8U,
				channels = 1
			};
		};

		template<> class DataType < unsigned char >
		{
		public:
			typedef unsigned char value_type;
			typedef value_type  channel_type;
			enum {
				depth = IMAGE_DEPTH_8U,
				channels = 1
			};
		};



		template<> class DataType < unsigned short >
		{
		public:
			typedef unsigned short      value_type;
			typedef value_type  channel_type;
			enum {
				depth = IMAGE_DEPTH_16U,
				channels = 1
			};
		};


			template<> class DataType < float >
			{
			public:
				typedef float       value_type;
				typedef value_type  channel_type;
				enum {
					depth = IMAGE_DEPTH_32F,
					channels = 1
				};
			};

	}
}

#endif //FBLIB_IMAGE_TRAITS_H_