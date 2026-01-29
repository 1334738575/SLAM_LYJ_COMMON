#ifndef COMMON_LYJ_ADJOINT_FINDER_H
#define COMMON_LYJ_ADJOINT_FINDER_H

#include "base/Base.h"
#include <opencv2/core.hpp>

namespace COMMON_LYJ
{

    template<typename T>
    class AdjointFinder
    {
    public:
        AdjointFinder() {}
        virtual ~AdjointFinder() {}

        void findAdjoint(const T& _p, std::vector<T>& _contour)
        {
            _contour.clear();
            if (!isValid(_p))
                return;
            std::stack<T> stk;
            stk.push(_p);
            setUnValid(_p);
            _contour.push_back(_p);
            while (!stk.empty())
                addP(stk, _contour);
        }

        //搜索全部周围点，建议使用findAdjoint，手动去除第一个点（自身），轮数过少不建议使用TODO
        void findAround(const T& _p, std::vector<T>& _pArounds, int _round)
        {
            _pArounds.clear();
            std::stack<T> stk;
            stk.push(_p);
            setUnValid(_p);
            static std::vector<T> pAround;
            static T p;
            int cnt = 0;
            while (!stk.empty() && cnt++ < _round)
            {
                int sz = stk.size();
                for (int i = 0; i < sz; ++i)
                    addP(stk, _pArounds);
            }
        }

        void findMultiAdjoint(const std::vector<T>& _sds, std::vector<std::vector<T>>& _contours)
        {
            int sz = _sds.size();
            std::vector<T> contour;
            for (int i = 0; i < sz; ++i)
            {
                const T& sd = _sds[i];
                findAdjoint(sd, contour);
                if (!contour.empty())
                    _contours.push_back(contour);
            }
        }


    protected:
        virtual void getAround(const T& _p, std::vector<T>& _psAround) = 0;
        virtual bool isValid(const T& _p) = 0;
        virtual void setUnValid(const T& _p) = 0;
        void addP(std::stack<T>& _stk, std::vector<T>& _ret)
        {
            static std::vector<T> pAround;
            static T p;
            p = _stk.top();
            _stk.pop();
            getAround(p, pAround);
            for (int ii = 0; ii < pAround.size(); ++ii)
            {
                const T& pTmp = pAround[ii];
                if (!isValid(pTmp))
                    continue;
                _stk.push(pTmp);
                setUnValid(pTmp);
                _ret.push_back(pTmp);
            }
        }
    };

    /// <summary>
    /// 简单的opencv二维邻接搜索类，只接受八位单通道，有效值为1，无效值为0
    /// </summary>
    class SLAM_LYJ_API AdjointFinderCV2DSimple : public AdjointFinder<cv::Point2i>
    {
    public:
        AdjointFinderCV2DSimple();
        ~AdjointFinderCV2DSimple() override;

        void setData(const cv::Mat& _img);

    protected:
        // 通过 AdjointFinder 继承
        void getAround(const cv::Point2i& _p, std::vector<cv::Point2i>& _psAround) override;
        bool isValid(const cv::Point2i& _p) override;
        void setUnValid(const cv::Point2i& _p) override;

        bool checkP(const cv::Point2i& _p);

        cv::Mat mask_;
    };
}

#endif //COMMON_LYJ_ADJOINT_FINDER_H
