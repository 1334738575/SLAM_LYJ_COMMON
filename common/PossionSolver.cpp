#include "PossionSolver.h"

namespace COMMON_LYJ
{
	ImagePossionSolver::ImagePossionSolver() {};
	ImagePossionSolver::~ImagePossionSolver() {};
	void ImagePossionSolver::getDDX(const Eigen::Vector2i& _l, std::vector<Eigen::Triplet<double>>& _vs, bool _updateVs, Eigen::Vector3d& _ddX)
	{
		int x = _l(0);
		int y = _l(1);
		_ddX.setZero();
		if (mask_->at<uchar>(y, x) == 0)
		{
			if (_updateVs)
			{
				_vs.emplace_back(loc2Inds_[y * src_->cols + x], loc2Inds_[y * src_->cols + x], 1);
			}
			const cv::Vec3b& v3Tmp = tgt_->at<cv::Vec3b>(y, x);
			for (int i = 0; i < 3; ++i)
				_ddX(i) = v3Tmp(i);
			return;
		}


		int coeff = 4;
		auto func = [&](const int& _x, const int& _y)
		{
			if (_x >= 0 && _x < src_->cols && _y >= 0 && _y < src_->rows)
			{
				if (_updateVs && mask_->at<uchar>(_y, _x) != 0)
				{
					_vs.emplace_back(loc2Inds_[y * src_->cols + x], loc2Inds_[_y * src_->cols + _x], -1);
				}
				if (mask_->at<uchar>(_y, _x) == 0)
				{
					const cv::Vec3b& v3Tmp = tgt_->at<cv::Vec3b>(_y, _x);
					for (int i = 0; i < 3; ++i)
						_ddX(i) += v3Tmp(i);
				}
			}
		};
		int lx = _l(0) - 1;
		func(lx, y);
		int rx = _l(0) + 1;
		func(rx, y);
		int uy = _l(1) - 1;
		func(x, uy);
		int dy = _l(1) + 1;
		func(x, dy);
		if (_updateVs)
		{
			_vs.emplace_back(loc2Inds_[y * src_->cols + x], loc2Inds_[y * src_->cols + x], coeff);
		}

		auto funcAdd = [](const cv::Mat& _img, const int& _x, const int& _y, Eigen::Vector3d& _ret)
		{
			const cv::Vec3b& v3Tmp = _img.at<cv::Vec3b>(_y, _x);
			for (int i = 0; i < 3; ++i)
				_ret(i) -= v3Tmp(i);
		};
		auto funcGetLaplas = [&](const cv::Mat& _im, Eigen::Vector3d& _r)
		{
			const cv::Vec3b& v3 = _im.at<cv::Vec3b>(y, x);
			for (int i = 0; i < 3; ++i)
				_r(i) += (coeff * (int)(v3(i)));
			funcAdd(_im, lx, y, _r);
			funcAdd(_im, rx, y, _r);
			funcAdd(_im, x, uy, _r);
			funcAdd(_im, x, dy, _r);

			//const cv::Vec3d& l = srcLap_.at<cv::Vec3d>(y, x);
			//for (int i = 0; i < 3; ++i)
			//	_ddX(i) += l(i);
		};
		Eigen::Vector3d rrrSrc = Eigen::Vector3d::Zero();
		Eigen::Vector3d rrrTgt = Eigen::Vector3d::Zero();
		funcGetLaplas(*src_, rrrSrc);
		funcGetLaplas(*tgt_, rrrTgt);
		for (int i = 0; i < 3; ++i)
		{
			//double d = std::abs(rrrSrc(i)) > std::abs(rrrTgt(i)) ? rrrSrc(i) : rrrTgt(i);
			double d = rrrSrc(i);
			_ddX(i) += d;
		}
		return;
	}
	void ImagePossionSolver::possionSolve(cv::Mat& _src, cv::Mat& _tgt, cv::Mat& _mask, cv::Point2i& _center)
	{
		vs_.clear();
		loc2Inds_.clear();
		if (_src.empty() || _tgt.empty())
			return;
		int sh = _src.rows;
		int sw = _src.cols;
		int th = _tgt.rows;
		int tw = _tgt.cols;
		src_ = &_src;
		cv::Rect rect(_center.x - sw / 2, _center.y - sh / 2, sw, sh);
		cv::Mat tgt = _tgt(rect);
		tgt_ = &tgt;
		//tgt_->setTo(cv::Scalar(0, 0, 0));
		//tgt_->setTo(cv::Scalar(128, 128, 128));
		//cv::Rect rectSrc(sw / 4, sh / 4, sw * 2 / 4, sh * 2 / 4);
		//cv::Mat maskSrc = _src(rectSrc);
		//src_->setTo(cv::Scalar(100, 100, 100));
		//maskSrc.setTo(cv::Scalar(255, 255, 255));
		if (_mask.empty()) {
			_mask = cv::Mat(_src.rows, _src.cols, CV_8UC1);
			_mask.setTo(0);
			cv::Rect rect2(1, 1, sw-2, sh-2);
			cv::Mat mask2 = _mask(rect2);
			mask2.setTo(255);
		}
		mask_ = &_mask;
		center_ = _center;
		//cv::Laplacian(*src_, srcLap_, CV_64F, 3, 1, 0, cv::BORDER_DEFAULT);
		//cv::Laplacian(*tgt_, tgtLap_, CV_64F, 3, 1, 0, cv::BORDER_DEFAULT);
		//cv::imshow("src", *src_);
		//cv::imshow("tgt", *tgt_);
		//cv::imshow("mask", *mask_);
		//cv::imwrite("D:/tmp/possion/mask.png", *mask_);
		//cv::waitKey();

		std::vector<Eigen::Vector2i> Locs;
		Locs.reserve(sh * sw);
		int cnt = 0;
		for (int r = 0; r < sh; ++r)
		{
			for (int c = 0; c < sw; ++c)
			{
				//if (mask_->at<uchar>(r, c) == 0)
					//continue;
				Locs.emplace_back(c, r);
				loc2Inds_[r * sw + c] = cnt;
				++cnt;
			}
		}

		for (int i = 0; i < 3; ++i)
		{
			Bs_[i].resize(cnt);
			Bs_[i].setZero();
			Xs_[i].resize(cnt);
			Xs_[i].setZero();
		}
		vs_.reserve(cnt * cnt);

		Eigen::Vector3d ddX;
		for (int i = 0; i < cnt; ++i)
		{
			getDDX(Locs[i], vs_, true, ddX);
			Bs_[0](i) = ddX(0);
			Bs_[1](i) = ddX(1);
			Bs_[2](i) = ddX(2);
		}

		if (!solve())
			return;

		for (int i = 0; i < cnt; ++i)
		{
			auto& v3 = tgt.at<cv::Vec3b>(Locs[i](1), Locs[i](0));
			for (int j = 0; j < 3; ++j) 
			{
				int x = Xs_[j](i) > 255 ? 255 : Xs_[j](i);
				x = x < 0 ? 0: x;
				//if (x > 255 || x < 0)
				//	std::cout << " 1111" << std::endl;
				v3(j) = (uchar)x;
			}
		}
		//cv::imshow("tgt2", tgt);
		//cv::waitKey();
	}

}