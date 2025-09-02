#ifndef COMMON_LYJ_SIMPLE_IO_H
#define COMMON_LYJ_SIMPLE_IO_H

#include <fstream>
#include <iostream>
#include <common/BaseTriMesh.h>
#include <IO/MeshIO.h>
#include "base/Pose.h"
#include "base/CameraModule.h"

namespace COMMON_LYJ
{
    bool readT34(const std::string &_file, SLAM_LYJ::Pose3D &_T)
    {
        std::ifstream f(_file);
        if (!f.is_open())
        {
            std::cout << "read pose failed!" << std::endl;
            return false;
        }
        f >> _T.getR()(0, 0) >> _T.getR()(0, 1) >> _T.getR()(0, 2) >> _T.gett()(0) >> _T.getR()(1, 0) >> _T.getR()(1, 1) >> _T.getR()(1, 2) >> _T.gett()(1) >> _T.getR()(2, 0) >> _T.getR()(2, 1) >> _T.getR()(2, 2) >> _T.gett()(2);
        f.close();
    }
    bool writeT34(const std::string &_file, const SLAM_LYJ::Pose3D &_T)
    {
        std::ofstream f(_file);
        if (!f.is_open())
        {
            std::cout << "read pose failed!" << std::endl;
            return false;
        }
        f << _T.getR()(0, 0) << " " << _T.getR()(0, 1) << " " << _T.getR()(0, 2) << " " << _T.gett()(0) << " " << _T.getR()(1, 0) << " " << _T.getR()(1, 1) << " " << _T.getR()(1, 2) << " " << _T.gett()(1) << " " << _T.getR()(2, 0) << " " << _T.getR()(2, 1) << " " << _T.getR()(2, 2) << " " << _T.gett()(2);
        f.close();
    }
    bool writePinCamera(const std::string &_filename, const SLAM_LYJ::PinHoleCmera &_cam)
    {
        std::ofstream of(_filename);
        if (!of.is_open())
            return false;
        of << _cam.wide() << " " << _cam.height() << " " << _cam.fx() << " " << _cam.fy() << " " << _cam.cx() << " " << _cam.cy() << std::endl;
        of.close();
        return true;
    }
    bool readPinCamera(const std::string &_filename, SLAM_LYJ::PinHoleCmera &_cam)
    {
        std::vector<double> params(4, 0);
        int w, h;
        std::ifstream f(_filename);
        if (!f.is_open())
            return false;
        f >> w >> h;
        for (int i = 0; i < 4; ++i)
            f >> params[i];
        f.close();
        _cam = SLAM_LYJ::PinCmera(w, h, params);
        return true;
    }
    bool drawCam(const std::string &_file, const SLAM_LYJ::PinholeCmera &_cam, const SLAM_LYJ::Pose3D &_Twc, const double _depth)
    {
        Eigen::Vector3d p1(0, 0, _depth);
        Eigen::Vector3d p2(_cam.wide(), 0, _depth);
        Eigen::Vector3d p3(_cam.wide(), _cam.height(), _depth);
        Eigen::Vector3d p4(0, _cam.height(), _depth);
        Eigen::Vector3d P0(0, 0, 0);
        Eigen::Vector3d P1;
        Eigen::Vector3d P2;
        Eigen::Vector3d P3;
        Eigen::Vector3d P4;
        _cam.image2World(p1, P1);
        _cam.image2World(p2, P2);
        _cam.image2World(p3, P3);
        _cam.image2World(p4, P4);
        P0 = _Twc * P0;
        P1 = _Twc * P1;
        P2 = _Twc * P2;
        P3 = _Twc * P3;
        P4 = _Twc * P4;
        SLAM_LYJ::SLAM_LYJ_MATH::BaseTriMesh btmTmp;
        std::vector<Eigen::Vector3f> psTmp;
        psTmp.push_back(P0.cast<float>());
        psTmp.push_back(P1.cast<float>());
        psTmp.push_back(P2.cast<float>());
        psTmp.push_back(P3.cast<float>());
        psTmp.push_back(P4.cast<float>());
        std::vector<SLAM_LYJ::SLAM_LYJ_MATH::BaseTriFace> fsTmp;
        fsTmp.emplace_back(0, 1, 2);
        fsTmp.emplace_back(0, 2, 3);
        fsTmp.emplace_back(0, 3, 4);
        fsTmp.emplace_back(0, 4, 1);
        btmTmp.setVertexs(psTmp);
        btmTmp.setFaces(fsTmp);
        SLAM_LYJ::writePLYMesh(_file, btmTmp);
        return true;
    }
    bool drawCoordinateSystem(const std::string &_file, const SLAM_LYJ::Pose3D &_Twc, const double _step)
    {
        int num = 5;
        Eigen::Vector3d P0(0, 0, 0);
        Eigen::Vector3d P1(_step, 0, 0);
        Eigen::Vector3d P2(0, _step, 0);
        Eigen::Vector3d P3(0, 0, _step);
        P0 = _Twc * P0;
        P1 = _Twc * P1;
        P2 = _Twc * P2;
        P3 = _Twc * P3;
        SLAM_LYJ::SLAM_LYJ_MATH::BaseTriMesh btmTmp;
        std::vector<Eigen::Vector3f> psTmp;
        std::vector<Eigen::Vector3f> clrsTmp;
        psTmp.push_back(P0.cast<float>());
        clrsTmp.push_back(Eigen::Vector3f(1, 1, 1));
        for (int i = 0; i < num; ++i)
        {
            psTmp.push_back(P1.cast<float>() + (P1.cast<float>() - P0.cast<float>()) * i);
            psTmp.push_back(P2.cast<float>() + (P2.cast<float>() - P0.cast<float>()) * i);
            psTmp.push_back(P3.cast<float>() + (P3.cast<float>() - P0.cast<float>()) * i);
            clrsTmp.push_back(Eigen::Vector3f(1, 0, 0));
            clrsTmp.push_back(Eigen::Vector3f(0, 1, 0));
            clrsTmp.push_back(Eigen::Vector3f(0, 0, 1));
        }
        btmTmp.setVertexs(psTmp);
        btmTmp.enableVColors();
        btmTmp.setVColors(clrsTmp);
        SLAM_LYJ::writePLYMesh(_file, btmTmp);
        return true;
    }
    bool drawCube(const std::string &_file, float _l, float _w, float _h)
    {
        float L = _l / 2;
        float W = _w / 2;
        float H = _h / 2;
        std::vector<Eigen::Vector3f> vertices;
        vertices.emplace_back(-L, -W, -H);
        vertices.emplace_back(L, -W, -H);
        vertices.emplace_back(L, W, -H);
        vertices.emplace_back(-L, W, -H);
        vertices.emplace_back(-L, -W, H);
        vertices.emplace_back(L, -W, H);
        vertices.emplace_back(L, W, H);
        vertices.emplace_back(-L, W, H);
        std::vector<SLAM_LYJ::BaseTriFace> faces;
        faces.emplace_back(0, 1, 2);
        faces.emplace_back(0, 2, 3);
        faces.emplace_back(4, 5, 6);
        faces.emplace_back(4, 6, 7);
        faces.emplace_back(2, 3, 7);
        faces.emplace_back(2, 7, 6);
        faces.emplace_back(0, 1, 5);
        faces.emplace_back(0, 5, 4);
        faces.emplace_back(0, 3, 7);
        faces.emplace_back(0, 7, 4);
        faces.emplace_back(1, 5, 6);
        faces.emplace_back(1, 6, 2);
        SLAM_LYJ::BaseTriMesh btm;
        btm.setVertexs(vertices);
        btm.setFaces(faces);
        SLAM_LYJ::writePLYMesh(_file, btm);
    }
} // namespace COMMON_LYJ

#endif // COMMON_LYJ_SIMPLE_IO_H