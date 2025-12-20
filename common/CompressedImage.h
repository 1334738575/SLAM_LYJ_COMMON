#ifndef COMMON_LYJ_COMPRESSEDIMAGE_H
#define COMMON_LYJ_COMPRESSEDIMAGE_H


#include <vector>
#include <opencv2/core.hpp>
#include "base/Base.h"


namespace COMMON_LYJ
{
	class SLAM_LYJ_API CompressedImage
	{
	public:
		CompressedImage();
		~CompressedImage();

		inline int getWidth() const { return w_; };
		inline int getheight() const { return h_; };
		inline int getChannels() const { return c_; };
		inline const std::vector<unsigned char>& getBinData() const { return binData_; };
		inline std::vector<unsigned char>& getBinData() { return binData_; };

		bool compress(const unsigned char* _data, const int _w, const int _h, const int _c, const int _quality=95);
		bool compressCVMat(const cv::Mat& _img);
		bool decompress(std::vector<unsigned char>& _data, int& _w, int& _h, int& _c);
		bool decompressCVMat(cv::Mat& _img);

		void write_binary(std::ofstream& os) const;
		void read_binary(std::ifstream& is);
	private:

		std::vector<unsigned char> binData_;
		int w_ = -1;
		int h_ = -1;
		int c_ = -1;
	};

}


#endif // !COMMON_LYJ_COMPRESSEDIMAGE_H
