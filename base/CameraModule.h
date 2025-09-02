#ifndef SLAM_LYJ_CAMERAMODULE_H
#define SLAM_LYJ_CAMERAMODULE_H

#include "Base.h"

NSP_SLAM_LYJ_BEGIN

enum CameraType
{
    DEFAULT,
    PINHOLE,
    FISHEYE
};
class CameraModule : public BaseLYJ
{
protected:
    /* data */
    CameraType type = DEFAULT;
    int w = -1;
    int h = -1;
    std::vector<double> params;

public:
    CameraModule(const CameraType _type, const int _w, const int _h, const std::vector<double> &_params)
        : type(_type), w(_w), h(_h), params(_params) {}
    CameraModule(const CameraType _type)
        : type(_type) {}
    ~CameraModule() {}

    // 物理到图像
    virtual void world2Image(const Eigen::Vector3d &_P, Eigen::Vector2d &_p) const = 0;
    virtual void world2Image(const Eigen::Vector3d &_P, double &_u, double &_v) const = 0;
    // 图像到物理
    virtual void image2World(const Eigen::Vector2d &_p, const double _d, Eigen::Vector3d &_P) const = 0;
    virtual void image2World(const double _u, const double _v, const double _d, Eigen::Vector3d &_P) const = 0;
    virtual void image2World(const Eigen::Vector3d &_p, Eigen::Vector3d &_P) const = 0; // u v d
    // 内参矩阵
    virtual Eigen::Matrix3d getK() const = 0;
    virtual const double &fx() const = 0;
    virtual const double &fy() const = 0;
    virtual const double &cx() const = 0;
    virtual const double &cy() const = 0;
    virtual double &fx() = 0;
    virtual double &fy() = 0;
    virtual double &cx() = 0;
    virtual double &cy() = 0;
    inline const CameraType getType() const
    {
        return type;
    }
    inline const int wide() const
    {
        return w;
    }
    inline const int height() const
    {
        return h;
    }
    bool inImage(const int _u, const int _v) const
    {
        if (_u < 0 || _u >= wide())
            return false;
        if (_v < 0 || _v >= height())
            return false;
        return true;
    }
};

class PinholeCmera : public CameraModule
{
public:
    PinholeCmera()
        : CameraModule(CameraType::PINHOLE)
    {
        params.assign(4, 0);
    }
    PinholeCmera(int _w, int _h, double _fx, double _fy, double _cx, double _cy)
        : CameraModule(CameraType::PINHOLE)
    {
        w = _w;
        h = _h;
        params = std::vector<double>{_fx, _fy, _cx, _cy};
    }
    PinholeCmera(int _w, int _h, const std::vector<double> &_params)
        : CameraModule(CameraType::PINHOLE, _w, _h, _params)
    {
    }
    PinholeCmera(const std::string &_path)
        : CameraModule(DEFAULT, -1, -1, std::vector<double>())
    {
        std::ifstream f(_path);
        if (!f.is_open())
        {
            std::cout << "Read pinhole camera file fail!" << std::endl;
            return;
        }
        params.resize(4);
        f >> w >> h >> params[0] >> params[1] >> params[2] >> params[3];
        type = PINHOLE;
        f.close();
    }
    ~PinholeCmera() {}

    void downSample()
    {
        w /= 2;
        h /= 2;
        for (auto &p : params)
            p /= 2;
    }
    PinholeCmera downSample() const
    {
        return PinholeCmera(w / 2, h / 2, fx() / 2, fy() / 2, cx() / 2, cy() / 2);
    }
    // 物理到图像
    // override
    void world2Image(const Eigen::Vector3d &_P, Eigen::Vector2d &_p) const
    {
        world2Image(_P, _p(0), _p(1));
    }
    // override
    void world2Image(const Eigen::Vector3d &_P, double &_u, double &_v) const
    {
        _u = fx() * _P(0) / _P(2) + cx();
        _v = fy() * _P(1) / _P(2) + cy();
    }
    // 图像到物理
    // override
    void image2World(const Eigen::Vector2d &_p, const double _d, Eigen::Vector3d &_P) const
    {
        image2World(_p(0), _p(1), _d, _P);
    }
    // override
    void image2World(const double _u, const double _v, const double _d, Eigen::Vector3d &_P) const
    {
        _P(0) = 1.0 / fx() * (_u - cx()) * _d;
        _P(1) = 1.0 / fy() * (_v - cy()) * _d;
        _P(2) = _d;
    }
    // override
    void image2World(const Eigen::Vector3d &_p, Eigen::Vector3d &_P) const
    {
        image2World(_p(0), _p(1), _p(2), _P);
    }
    // 内参矩阵
    // override
    Eigen::Matrix3d getK() const
    {
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = fx();
        K(1, 1) = fy();
        K(0, 2) = cx();
        K(1, 2) = cy();
        return K;
    }
    const double &fx() const
    {
        return params[0];
    }
    const double &fy() const
    {
        return params[1];
    }
    const double &cx() const
    {
        return params[2];
    }
    const double &cy() const
    {
        return params[3];
    }

    double &fx()
    {
        return params[0];
    }
    double &fy()
    {
        return params[1];
    }
    double &cx()
    {
        return params[2];
    }
    double &cy()
    {
        return params[3];
    }

    void write_binary(std::ofstream &os) override
    {
        os.write(reinterpret_cast<const char *>(&type), sizeof(int) * 1);
        os.write(reinterpret_cast<const char *>(&w), sizeof(int) * 1);
        os.write(reinterpret_cast<const char *>(&h), sizeof(int) * 1);
        os.write(reinterpret_cast<const char *>(params.data()), sizeof(double) * 4);
    }
    void read_binary(std::ifstream &is) override
    {
        is.read(reinterpret_cast<char *>(&type), sizeof(int) * 1);
        is.read(reinterpret_cast<char *>(&w), sizeof(int) * 1);
        is.read(reinterpret_cast<char *>(&h), sizeof(int) * 1);
        is.read(reinterpret_cast<char *>(params.data()), sizeof(double) * 4);
    }
    friend std::ostream &operator<<(std::ostream &os, const PinholeCmera &cls)
    {
        std::cout << cls.getType() << std::endl;
        std::cout << cls.getK();
        return os;
    }

private:
};

NSP_SLAM_LYJ_END

#endif // SLAM_LYJ_CAMERAMODULE_H