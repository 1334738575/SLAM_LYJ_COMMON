#ifndef COMMON_LYJ_SIMPLE_IO_H
#define COMMON_LYJ_SIMPLE_IO_H

#include <fstream>
#include <iostream>
#include "common/BaseTriMesh.h"
#include "MeshIO.h"
#include "base/Pose.h"
#include "base/CameraModule.h"

namespace COMMON_LYJ
{
    SLAM_LYJ_API bool readT34(const std::string& _file, SLAM_LYJ::Pose3D& _T);
    SLAM_LYJ_API bool writeT34(const std::string& _file, const SLAM_LYJ::Pose3D& _T);
    SLAM_LYJ_API bool writePinCamera(const std::string& _filename, const SLAM_LYJ::PinholeCamera& _cam);
    SLAM_LYJ_API bool readPinCamera(const std::string& _filename, SLAM_LYJ::PinholeCamera& _cam);
    SLAM_LYJ_API bool drawCam(const std::string& _file, const SLAM_LYJ::PinholeCamera& _cam, const SLAM_LYJ::Pose3D& _Twc, const double _depth);
    SLAM_LYJ_API bool drawCoordinateSystem(const std::string& _file, const SLAM_LYJ::Pose3D& _Twc, const double _step);
    SLAM_LYJ_API bool drawCube(const std::string& _file, float _l, float _w, float _h);
} // namespace COMMON_LYJ

#endif // COMMON_LYJ_SIMPLE_IO_H