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
		const cv::Vec3b& v3 = src_->at<cv::Vec3b>(y, x);
		if (mask_->at<uchar>(y, x) == 0)
		{
			_vs.emplace_back(loc2Inds_[_l(1) * src_->cols + _l(0)], loc2Inds_[_l(1) * src_->cols + _l(0)], 1);
			const cv::Vec3b& v3Tmp = tgt_->at<cv::Vec3b>(y, x);
			for (int i = 0; i < 3; ++i)
				_ddX(i) = v3Tmp(i);
			return;
		}
		int coeff = -4;

		auto func = [&](const int& _x, const int& _y) 
		{
			if (x >= 0 && x < src_->cols && y >= 0 && y < src_->rows)
			{
				if (mask_->at<uchar>(y, x) != 0)
				{
					const cv::Vec3b& v3Tmp = src_->at<cv::Vec3b>(y, x);
					for (int i = 0; i < 3; ++i)
						_ddX(i) += v3Tmp(i);
					_vs.emplace_back(loc2Inds_[_l(1) * src_->cols + _l(0)], loc2Inds_[y * src_->cols + x], 1);
				}
				else
				{
					const cv::Vec3b& v3Tmp = tgt_->at<cv::Vec3b>(y, x);
					for (int i = 0; i < 3; ++i)
						_ddX(i) += v3Tmp(i);
					_vs.emplace_back(loc2Inds_[_l(1) * src_->cols + _l(0)], loc2Inds_[y * src_->cols + x], 1);
					//const cv::Vec3b& v3Tmp = src_->at<cv::Vec3b>(y, x);
					//const cv::Vec3b& v3Tmp2 = tgt_->at<cv::Vec3b>(y, x);
					//for (int i = 0; i < 3; ++i) {
					//	_ddX(i) += v3Tmp(i);
					//	_ddX(i) -= v3Tmp2(i);
					//}
					//++coeff;
				}
			}
		};
		x = _l(0) - 1;
		y = _l(1);
		func(x, y);
		x = _l(0) + 1;
		y = _l(1);
		func(x, y);
		x = _l(0);
		y = _l(1) - 1;
		func(x, y);
		x = _l(0);
		y = _l(1) + 1;
		func(x, y);
		for (int i = 0; i < 3; ++i)
			_ddX(i) += (coeff * (int)(v3(i)));
		_vs.emplace_back(loc2Inds_[_l(1) * src_->cols + _l(0)], loc2Inds_[_l(1) * src_->cols + _l(0)], coeff);
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
		if (_mask.empty()) {
			_mask = cv::Mat(_src.rows, _src.cols, CV_8UC1);
			_mask.setTo(0);
			cv::Rect rect2(1, 1, sw-2, sh-2);
			cv::Mat mask2 = _mask(rect2);
			mask2.setTo(255);
		}
		mask_ = &_mask;
		center_ = _center;
		//cv::imshow("src", *src_);
		//cv::imshow("tgt", *tgt_);
		//cv::imshow("mask", *mask_);
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
				v3(j) = (uchar)x;
			}
		}
	}

}