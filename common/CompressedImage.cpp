#include "CompressedImage.h"
//#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "IO/BaseIO.h"

namespace COMMON_LYJ
{
	CompressedImage::CompressedImage()
	{
	}
	CompressedImage::~CompressedImage()
	{
	}
	bool CompressedImage::compress(const unsigned char* _data, const int _w, const int _h, const int _c, const int _quality)
	{
        if (_quality < 1 || _quality > 100) {
            throw std::runtime_error("压缩质量需在1-100之间");
            return false;
        }

        tjhandle tjCompressor = tjInitCompress();
        if (!tjCompressor) {
            throw std::runtime_error("初始化libjpeg-turbo压缩器失败");
            return false;
        }

        w_ = _w;
        h_ = _h;
        c_ = _c;

        unsigned char* jpegBuf = nullptr;
        unsigned long jpegSize = 0;
        int width = w_;
        int height = h_;
        int pitch = w_ * c_; // 每行字节数
        int pixelFormat;
        const unsigned char* srcData = _data;
        int subsamp; // 色度采样模式（灰度图专用）

        // 处理灰度图/彩色图
        if (c_ == 1) {
            pixelFormat = TJPF_GRAY;
            subsamp = TJSAMP_GRAY;
        }
        else if (c_ == 3) {
            pixelFormat = TJPF_RGB;
            subsamp = TJSAMP_420;
        }
        else {
            tjDestroy(tjCompressor);
            throw std::runtime_error("仅支持1通道（灰度）/3通道（彩色）Mat");
        }

        // 核心压缩API
        int ret = tjCompress2(
            tjCompressor,
            srcData,        // 输入像素数据
            width,          // 宽度
            pitch,          // 每行字节数（pitch）
            height,         // 高度
            pixelFormat,    // 像素格式（灰度/TJPF_GRAY，彩色/TJPF_RGB）
            &jpegBuf,       // 输出JPEG缓冲区（由turbojpeg分配）
            &jpegSize,      // 输出JPEG数据长度
            subsamp,     // 色度采样（420压缩率最高，444画质最好）
            _quality,        // 压缩质量
            TJFLAG_FASTDCT  // 快速DCT（牺牲少量画质提升速度）
        );

        if (ret != 0) {
            tjDestroy(tjCompressor);
            throw std::runtime_error("JPEG压缩失败：" + std::string(tjGetErrorStr()));
        }

        // 拷贝JPEG数据到vector并释放turbojpeg缓冲区
        binData_.resize(jpegSize);
        memcpy(binData_.data(), jpegBuf, jpegSize);
        tjFree(jpegBuf);
        tjDestroy(tjCompressor);

		return true;
	}
    bool CompressedImage::compressCVMat(const cv::Mat& _img)
    {
        if (_img.empty())
        {
            throw std::runtime_error("输入Mat数据为空");
            return false;
        }
        if (_img.type() == CV_8UC3)
        {
            cv::Mat matRGB;
            cv::cvtColor(_img, matRGB, cv::COLOR_BGR2RGB);
            return compress(matRGB.data, matRGB.cols, matRGB.rows, matRGB.channels());
        }
        else if (_img.type() == CV_8UC1)
        {
            return compress(_img.data, _img.cols, _img.rows, _img.channels());
        }
        else
        {
            throw std::runtime_error("输入Mat类型错误");
            return false;
        }
        return true;
    }
    bool CompressedImage::decompress(std::vector<unsigned char>& _data, int& _w, int& _h, int& _c)
    {
        if (binData_.empty()) {
            throw std::runtime_error("输入JPEG数据为空");
            return false;
        }

        tjhandle tjDecompressor = tjInitDecompress();
        if (!tjDecompressor) {
            throw std::runtime_error("初始化libjpeg-turbo解压器失败");
            return false;
        }

        _w = w_;
        _h = h_;
        _c = _c;
        // 第一步：获取JPEG头信息（宽高、格式等）
        int jpegSubsamp, jpegColorspace;
        int ret = tjDecompressHeader3(
            tjDecompressor,
            binData_.data(),
            binData_.size(),
            &w_,
            &h_,
            &jpegSubsamp,
            &jpegColorspace
        );
        if (ret != 0) {
            tjDestroy(tjDecompressor);
            throw std::runtime_error("解析JPEG头失败：" + std::string(tjGetErrorStr()));
            return false;
        }

        // 第二步：解压缩为像素数据（优先解压为RGB/灰度）
        int pixelFormat = (jpegColorspace == TJCS_GRAY) ? TJPF_GRAY : TJPF_RGB;
        c_ = (pixelFormat == TJPF_GRAY) ? 1 : 3;
        int pitch = w_ * c_;
        _data.resize(h_ * pitch);

        ret = tjDecompress2(
            tjDecompressor,
            binData_.data(),
            binData_.size(),
            _data.data(),
            w_,
            pitch,
            h_,
            pixelFormat,
            TJFLAG_FASTDCT // 快速逆DCT TJFLAG_FASTDCT
        );
        if (ret != 0) {
            tjDestroy(tjDecompressor);
            throw std::runtime_error("JPEG解压缩失败：" + std::string(tjGetErrorStr()));
            return false;
        }

        tjDestroy(tjDecompressor);
        return true;
    }
    bool CompressedImage::decompressCVMat(cv::Mat& _img)
    {
        if (binData_.empty()) {
            throw std::runtime_error("输入JPEG数据为空");
            return false;
        }

        tjhandle tjDecompressor = tjInitDecompress();
        if (!tjDecompressor) {
            throw std::runtime_error("初始化libjpeg-turbo解压器失败");
            return false;
        }

        // 第一步：获取JPEG头信息（宽高、格式等）
        int jpegSubsamp, jpegColorspace;
        int ret = tjDecompressHeader3(
            tjDecompressor,
            binData_.data(),
            binData_.size(),
            &w_,
            &h_,
            &jpegSubsamp,
            &jpegColorspace
        );
        if (ret != 0) {
            tjDestroy(tjDecompressor);
            throw std::runtime_error("解析JPEG头失败：" + std::string(tjGetErrorStr()));
            return false;
        }

        // 第二步：解压缩为像素数据（优先解压为RGB/灰度）
        int pixelFormat = (jpegColorspace == TJCS_GRAY) ? TJPF_GRAY : TJPF_RGB;
        c_ = (pixelFormat == TJPF_GRAY) ? 1 : 3;
        int pitch = w_ * c_;
        if (c_ == 3)
            _img = cv::Mat(h_, w_, CV_8UC3);
        else if (c_ == 1)
            _img = cv::Mat(h_, w_, CV_8UC1);
        else
        {
            throw std::runtime_error("通道错误");
            return false;
        }

        ret = tjDecompress2(
            tjDecompressor,
            binData_.data(),
            binData_.size(),
            _img.data,
            w_,
            pitch,
            h_,
            pixelFormat,
            TJFLAG_FASTDCT // 快速逆DCT TJFLAG_FASTDCT
        );
        if (ret != 0) {
            tjDestroy(tjDecompressor);
            throw std::runtime_error("JPEG解压缩失败：" + std::string(tjGetErrorStr()));
            return false;
        }


        // 转换为cv::Mat（彩色图需RGB→BGR）
        if (c_ == 3) {
            cv::cvtColor(_img, _img, cv::COLOR_RGB2BGR);
        }

        tjDestroy(tjDecompressor);
        return true;
    }



    void CompressedImage::write_binary(std::ofstream& os) const
    {
        COMMON_LYJ::writeBinDatas<const int&, const int&, const int&, const std::vector<unsigned char>&>(os, w_, h_, c_, binData_);
    }
    void CompressedImage::read_binary(std::ifstream& is)
    {
        COMMON_LYJ::readBinDatas<int, int, int, std::vector<unsigned char>>(is, w_, h_, c_, binData_);
    }
}
