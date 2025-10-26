#include "RANSAC.h"
#include "common/Plane.h"

NSP_SLAM_LYJ_MATH_BEGIN


RANSACLine3f::RANSACLine3f(const double _errTh, const double _lenTh, 
	const double _inlineRatioTh, const int _minInlineNum,
	const int _maxIterNum, const double _dstInlineRatioTh, const double _stopRatio)
	:RANSAC<Eigen::Vector3f, float, Line3f>(_inlineRatioTh, _minInlineNum, _maxIterNum, _dstInlineRatioTh, _stopRatio), m_errTh(_errTh), m_lenTh(_lenTh)
{
	double dstRat = std::clamp(_dstInlineRatioTh, 0.01, 0.99);
	double invInRatN = 1 - std::pow(_inlineRatioTh, _minInlineNum);
	double invDstRat = 1 - dstRat;
	int maxInterNum = static_cast<int>(std::log(invDstRat) / std::log(invDstRat));
	m_maxIterNum = std::min(_maxIterNum, maxInterNum);
	m_funcCalErr = std::bind(&RANSACLine3f::calErr, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	m_funcCalModule = std::bind(&RANSACLine3f::calMdl, this, std::placeholders::_1, std::placeholders::_2);
};
bool RANSACLine3f::calErr(const Line3f& _L3D, const Eigen::Vector3f& _P, float& _err)
{
	_err = _L3D.distP2L(_P);
	if (_err > 0.01)
		return false;
	return true;
}
bool RANSACLine3f::calMdl(const std::vector<const Eigen::Vector3f*>& _samples, Line3f& _mdl)
{
	_mdl.update(*(_samples[0]), *(_samples[1]));
	return _mdl.length > 0;
}



bool RANSACLine3DWithLine2D::getPlaneFromLine2D(const Pose3D& _Twc, const Eigen::Vector4d& _l2d, const PinholeCamera& _cam, Eigen::Vector4d& _plane)
{
    _plane.setZero();
    const Eigen::Vector2d& sp = _l2d.head<2>();
    const Eigen::Vector2d& ep = _l2d.tail<2>();
    Eigen::Vector3d spn;
    Eigen::Vector3d epn;
    _cam.image2World(sp, 10.0, spn);
    _cam.image2World(ep, 10.0, epn);
    Eigen::Vector3d pn1 = _Twc * spn;
    Eigen::Vector3d pn2 = _Twc * epn;
    Eigen::Vector3d pn3 = _Twc.gett();
    Plane3d pln(pn1, pn2, pn3);
    if (!pln.isValid())
        return false;
    _plane = pln.params;
    return true;
}

bool RANSACLine3DWithLine2D::triLine3DWithLine2D(
    const Pose3D& _Twc1, const Eigen::Vector4d& _l2d1, const PinholeCamera& _cam1, 
    const Pose3D& _Twc2, const Eigen::Vector4d& _l2d2, const PinholeCamera& _cam2, 
    MDL& _plk)
{
    _plk.setZero();
    Eigen::Vector4d plane1;
    getPlaneFromLine2D(_Twc1, _l2d1, _cam1, plane1);
    Eigen::Vector4d plane2;
    getPlaneFromLine2D(_Twc2, _l2d2, _cam2, plane2);
    Eigen::Matrix<double, 6, 1> plkw = Line3d::pipi_plk(plane1, plane2);
    if (plkw.tail<3>().squaredNorm() < 1e-6)
        return false;
    _plk = plkw;
	return true;
}

int RANSACLine3DWithLine2D::calErr(const MDL& _mdl, const std::vector<int>& _datas, std::vector<T>& _errs, std::vector<bool>& _bInls)
{
	int validCnt = 0;
	_errs.assign(_datas.size(), -1);
	_bInls.assign(_datas.size(), false);
	for (size_t i = 0; i < _datas.size(); ++i)
	{
		const auto& data = m_datas[_datas[i]];
		const Pose3D Tcw = m_Tcws[_datas[i]];
		Eigen::Matrix<double, 6, 1> plkc = Line3d::plk_to_pose(_mdl, Tcw.getR(), Tcw.gett());
		Eigen::Vector3d l2d = m_KKs[_datas[i]] * plkc.head<3>();
		const auto& ob = data.second;
		const auto& ps = m_datas[_datas[i]].second;
		double e1 = ps(0) * l2d(0) + ps(1) * l2d(1) + l2d(2);
		double e2 = ps(2) * l2d(0) + ps(3) * l2d(1) + l2d(2);
		double l_sqrtnorm = sqrt(l2d(0) * l2d(0) + l2d(1) * l2d(1));
		_errs[i] = (e1 + e2) / l_sqrtnorm;
		if (_errs[i] < m_errTh)
		{
			_bInls[i] = true;
			validCnt++;
		}
		else
		{
			_bInls[i] = false;
		}
	}
	return validCnt;
}

bool RANSACLine3DWithLine2D::calMdl(const std::vector<int>& _samples, MDL& _mdl)
{
	_mdl.setZero();
	if (_samples.size() < 2)
		return false;
	if(!triLine3DWithLine2D(m_datas[_samples[0]].first, m_datas[_samples[0]].second, m_cams[_samples[0]],
		m_datas[_samples[1]].first, m_datas[_samples[1]].second, m_cams[_samples[1]],
		_mdl))
		return false;
	return true;
}



NSP_SLAM_LYJ_MATH_END
