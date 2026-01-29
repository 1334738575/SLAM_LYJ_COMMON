#include "AdjointFinder.h"


namespace COMMON_LYJ
{
    AdjointFinderCV2DSimple::AdjointFinderCV2DSimple()
    {}
    AdjointFinderCV2DSimple::~AdjointFinderCV2DSimple()
    {}

    void AdjointFinderCV2DSimple::setData(const cv::Mat& _img)
    {
        mask_ = _img.clone();
    }

    // Í¨¹ý AdjointFinder ¼Ì³Ð
    void AdjointFinderCV2DSimple::getAround(const cv::Point2i& _p, std::vector<cv::Point2i>& _psAround)
    {
        _psAround.resize(8);
        const int& x = _p.x;
        const int& y = _p.y;
        _psAround[0].x = x - 1;
        _psAround[0].y = y - 1;
        _psAround[1].x = x;
        _psAround[1].y = y - 1;
        _psAround[2].x = x + 1;
        _psAround[2].y = y - 1;
        _psAround[3].x = x - 1;
        _psAround[3].y = y;
        _psAround[4].x = x + 1;
        _psAround[4].y = y;
        _psAround[5].x = x - 1;
        _psAround[5].y = y + 1;
        _psAround[6].x = x;
        _psAround[6].y = y + 1;
        _psAround[7].x = x + 1;
        _psAround[7].y = y + 1;
        //for (int i = -1; i <= 1; ++i)
        //{
        //    for (int j = -1; j <= 1; ++j)
        //    {
        //        if (i == 0 && j == 0)
        //            continue;

        //    }
        //}
    }
    bool AdjointFinderCV2DSimple::isValid(const cv::Point2i& _p)
    {
        if (!checkP(_p))
            return false;
        if (mask_.at<uchar>(_p.y, _p.x) == 0)
            return false;
        return true;
    }
    void AdjointFinderCV2DSimple::setUnValid(const cv::Point2i& _p)
    {
        if (!checkP(_p))
            return;
        mask_.at<uchar>(_p.y, _p.x) = 0;
    }

    bool AdjointFinderCV2DSimple::checkP(const cv::Point2i& _p)
    {
        if (mask_.empty())
            return false;
        if (_p.x < 0 || _p.x >= mask_.cols || _p.y < 0 || _p.y >= mask_.rows)
            return false;
        return true;
    }
}