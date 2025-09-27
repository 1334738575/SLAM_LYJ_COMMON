#include "RANSAC.h"

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




NSP_SLAM_LYJ_MATH_END
