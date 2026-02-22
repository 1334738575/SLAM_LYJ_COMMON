#ifndef SLAM_LYJ_CYLINDER_H
#define SLAM_LYJ_CYLINDER_H

#include "base/Base.h"
#include "base/Pose.h"


namespace COMMON_LYJ
{
    template<typename T>
    struct Cylinder
    {
        //using TYPE = double;
        using TemVec2 = Eigen::Matrix<T, 2, 1>;
        using TemVec3 = Eigen::Matrix<T, 3, 1>;
        using TemVec4 = Eigen::Matrix<T, 4, 1>;
        using TemMat22 = Eigen::Matrix<T, 2, 2>;
        using TemMat23 = Eigen::Matrix<T, 2, 3>;
        using TemMat33 = Eigen::Matrix<T, 3, 3>;
        using TemMat34 = Eigen::Matrix<T, 3, 4>;
        using TemMat44 = Eigen::Matrix<T, 4, 4>;

        T len_;
        T radius;


        Cylinder() {};

    };


}

#endif