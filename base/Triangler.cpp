#include "Triangler.h"
#include "common/RANSAC.h"


namespace COMMON_LYJ
{

    bool TrianglerPoint3D::runDirect(const std::vector<Eigen::Vector2d>& _uvs, const std::vector<Pose3D>& _Tcws, const std::vector<PinholeCamera>& _cams, Eigen::Vector3d& _Pw, TrianglerPoint3DOption _opt)
    {
        // construct
        Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
        Eigen::Vector3d point;
        Eigen::Matrix<double, 3, 4> Tcw;
        Eigen::Matrix<double, 3, 4> term;
        for (size_t i = 0; i < _uvs.size(); i++)
        {
            _cams[i].image2World(_uvs[i], 1, point);
            point.normalize();
            Tcw.block(0, 0, 3, 3) = _Tcws[i].getR();
            Tcw.block(0, 3, 3, 1) = _Tcws[i].gett();
            term = Tcw - point * point.transpose() * Tcw;
            A += term.transpose() * term;
        }
        // solve
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(A);
        _Pw = eigen_solver.eigenvectors().col(0).hnormalized();
        // check
        Eigen::Vector2d uvTmp;
        Eigen::Vector3d Pc;
        for (size_t i = 0; i < _uvs.size(); ++i)
        {
            Pc = _Tcws[i] * _Pw;
            if (Pc(2) <= 0)
                return false;
            //_cams[i].world2Image(Pc, uvTmp);
            //if ((uvTmp - _uvs[i]).norm() > _opt.inlineErrTh)
            //    return false;
        }
        return true;
    }

    bool TrianglerPoint3D::runRANSAC(const std::vector<Eigen::Vector2d>& _uvs, const std::vector<Pose3D>& _Tcws, const std::vector<PinholeCamera>& _cams, Eigen::Vector3d& _Pw, TrianglerPoint3DOption _opt)
    {
        std::vector<std::pair<Pose3D, Eigen::Vector2d>> datas(_uvs.size());
        for (size_t i = 0; i < _uvs.size(); ++i)
        {
            datas[i].first = _Tcws[i];
            datas[i].second = _uvs[i];
        }
        const std::vector<PinholeCamera>& cams = _cams;
        const double& errTh = _opt.inlineErrTh;
        const double& preInlineRatio = _opt.ransacPriorInlineRatio;
        const int& minNum2Solve = _opt.ransacMinInlineNum;
        const int& maxIterNum = _opt.ransacMaxItrtNum;
        const double& dstSampleRatioTh = _opt.ransacDstRatioTh;
        const double& minRatio = _opt.minRatio;

        RANSACPoint3DWithPoint2D ransac(datas, cams, errTh, preInlineRatio, maxIterNum, dstSampleRatioTh, minRatio);
        std::vector<double> errs;
        std::vector<bool> bInlines;
        std::vector<int> bestSample;
        double r = ransac.run(datas.size(), errs, bInlines, _Pw, bestSample);
        if (r < _opt.minRatio)
            return false;
        return true;
    }




    bool TrianglerLine3D::runRANSAC(const std::vector<Eigen::Vector4d>& _l2ds, const std::vector<Pose3D>& _Tcws, const std::vector<PinholeCamera>& _cams, Eigen::Matrix<double, 6, 1>& _plk, TrianglerLine3DOption _opt)
    {
        std::vector<std::pair<Pose3D, Eigen::Vector4d>> datas(_l2ds.size());
        for (size_t i = 0; i < _l2ds.size(); ++i)
        {
            datas[i].first = _Tcws[i];
            datas[i].second = _l2ds[i];
        }
        const std::vector<PinholeCamera>& cams = _cams;
        const double& errTh = _opt.inlineErrTh;
        const double& preInlineRatio = _opt.ransacPriorInlineRatio;
        const int& minNum2Solve = _opt.ransacMinInlineNum;
        const int& maxIterNum = _opt.ransacMaxItrtNum;
        const double& dstSampleRatioTh = _opt.ransacDstRatioTh;
        const double& minRatio = _opt.minRatio;

        RANSACLine3DWithLine2D ransac(datas, cams, errTh, preInlineRatio, maxIterNum, dstSampleRatioTh, minRatio);
        std::vector<double> errs;
        std::vector<bool> bInlines;
        std::vector<int> bestSample;
        double r = ransac.run(datas.size(), errs, bInlines, _plk, bestSample);
        if (r < _opt.minRatio)
            return false;
        return true;
    }


}

