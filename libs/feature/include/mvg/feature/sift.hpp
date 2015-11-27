#ifndef MVG_FEATURE_SIFT_HPP_
#define MVG_FEATURE_SIFT_HPP_

#include <iostream>
#include <numeric>

#include "mvg/feature/feature.h"
#include "mvg/feature/descriptor.h"
#include "mvg/image/image.h"

extern "C" {
#include "sift.h"
}

namespace mvg {
	namespace feature{

		using namespace mvg::image;

		// RootSift from [1]
		// [1] R. Arandjelovi膰, A. Zisserman.
		// Three things everyone should know to improve object retrieval. CVPR2012.
		// -> rootsift= sqrt( sift / sum(sift) );

		inline void siftDescToFloat(vl_sift_pix descr[128],
			Descriptor<float, 128> & descriptor, bool brootSift = false)
		{
			//rootsift= sqrt( sift / sum(sift) );
			if (brootSift)  {
				float sum = std::accumulate(descr, descr + 128, 0.0f);
				for (int k = 0; k < 128; ++k)
					descriptor[k] = sqrt(descr[k] / sum);
			}
			else
				for (int k = 0; k < 128; ++k)
					descriptor[k] = descr[k];
		}


		inline void siftDescToFloat(vl_sift_pix descr[128],
			Descriptor<unsigned char, 128> & descriptor, bool brootSift = false)
		{
			//rootsift= sqrt( sift / sum(sift) );
			if (brootSift)  {
				float sum = std::accumulate(descr, descr + 128, 0.0f);
				for (int k = 0; k < 128; ++k)
					descriptor[k] = static_cast<unsigned char>(512.f*sqrt(descr[k] / sum));
			}
			else
				for (int k = 0; k < 128; ++k)
					descriptor[k] = static_cast<unsigned char>(512.f*descr[k]);
		}

		/**
		 * \brief	Sift鐗瑰緛妫€娴?
		 *
		 * \tparam	type	鎻忚堪瀛愮被鍨?
		 * \param	image			  	杈揿叆锲惧儚
		 * \param [in,out]	feats	  	杈揿嚭鐗瑰緛
		 * \param [in,out]	descs	  	杈揿嚭鐗瑰緛鎻忚堪瀛?
		 * \param	is_zoom			  	鏄惁灏嗙涓€缁勭储寮曞€艰涓?1锛屼负-1锛屽垯锲惧儚鍦ㄨ绠楅佩鏂昂搴︾┖闂翠箣鍓嶅厛灏嗗昂搴︽墿澶т竴链?
		 * \param	is_root_sift	  	鏄惁浣跨敤root sift鏂规硶璁＄畻鐗瑰緛.
		 * \param	contrast_threshold	DoG绠楀瓙镄勫搷搴斿€?
		 *
		 * \return	true if it succeeds, false if it fails.
		 */
		template<typename type>
		static bool SIFTDetector(const Image<unsigned char>& image,
			std::vector<ScalePointFeature>& feats,
			std::vector<Descriptor<type, 128> >& descs,
			bool is_zoom = false,
			bool is_root_sift = false,
			float contrast_threshold = 0.04f)
		{
			// 绗竴缁勭殑绱㈠紩锛屽綋链间负-1锛屽垯锲惧儚鍦ㄨ绠楅佩鏂昂搴︾┖闂翠箣鍓嶅厛灏嗗昂搴︽墿澶т竴链?
			int first_octave = (is_zoom == true) ? -1 : 0;
			// 缁勭殑鏁扮洰
			int num_octaves = 6;
			// 姣忕粍灞傛暟镄勬暟鐩?
			int num_scales = 3;
			// 鐭╅樀链€澶х壒寰佸€间笌链€灏忕壒寰佸€肩殑姣旗巼
			float edge_thresh = 10.0f;
			// DoG链€灏忓搷搴斿€?
			float contrast_thresh = contrast_threshold;

			int w = image.Width(), h = image.Height();
			//杞崲涓烘诞镣圭被鍨嫔浘镀?
			Image<float> float_image(image.GetMat().cast<float>());

			vl_constructor();

			VlSiftFilt *filt = vl_sift_new(w, h, num_octaves, num_scales, first_octave);
			if (edge_thresh >= 0)
				vl_sift_set_edge_thresh(filt, edge_thresh);
			if (contrast_thresh >= 0)
				vl_sift_set_peak_thresh(filt, 255 * contrast_thresh / num_scales);

			vl_sift_process_first_octave(filt, float_image.data());

			vl_sift_pix descr[128];
			Descriptor<type, 128> descriptor;

			while (true) {
				vl_sift_detect(filt);

				VlSiftKeypoint const *keys = vl_sift_get_keypoints(filt);
				int nkeys = vl_sift_get_nkeypoints(filt);

				for (int i = 0; i < nkeys; ++i) {
					double angles[4];
					int	nangles = vl_sift_calc_keypoint_orientations(filt, angles, keys + i);

					for (int q = 0; q < nangles; ++q) {
						vl_sift_calc_keypoint_descriptor(filt, descr, keys + i, angles[q]);
						ScalePointFeature fp;
						fp.x() = keys[i].x;
						fp.y() = keys[i].y;
						fp.scale() = keys[i].sigma;
						fp.orientation() = static_cast<float>(angles[q]);

						siftDescToFloat(descr, descriptor, is_root_sift);
						descs.push_back(descriptor);
						feats.push_back(fp);
					}
				}
				if (vl_sift_process_next_octave(filt))
					break; // Last octave
			}
			vl_sift_delete(filt);

			vl_destructor();

			return true;
		}

	}// namespace feature
} // namespace mvg

#endif // MVG_FEATURE_SIFT_H_
