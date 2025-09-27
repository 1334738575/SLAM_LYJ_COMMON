#include "CommonAlgorithm.h"

NSP_SLAM_LYJ_MATH_BEGIN

bool SLAM_LYJ_API computeLineByPCA(const std::vector<Eigen::Vector3d>& _pts, Eigen::Vector3d& _p0, Eigen::Vector3d& _dir)
{
	if (_pts.size() < 2)
		return false;
	Eigen::Vector3d center = Eigen::Vector3d::Zero();
	for (const auto& p : _pts)
		center += p;
	center /= static_cast<double>(_pts.size());
	Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
	Eigen::Vector3d tmp;
	for (const auto& p : _pts)
	{
		tmp = p - center;
		cov += tmp * tmp.transpose();
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
	_dir = es.eigenvectors().col(2).normalized();
	_p0 = center;
	return true;
}

bool SLAM_LYJ_API computePlaneByPCA(const std::vector<Eigen::Vector3d>& _pts, Eigen::Vector3d& _n, Eigen::Vector3d& _p0)
{
	if (_pts.size() < 3)
		return false;

	int num_points = _pts.size();

	// 1. 计算质心
	_p0 = Eigen::Vector3d(0, 0, 0);
	for (const auto& p : _pts) _p0 += p;
	_p0 /= num_points;

	// 2. 构建协方差矩阵
	Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
	for (const auto& p : _pts) {
		Eigen::Vector3d adjusted = p - _p0;
		covariance += adjusted * adjusted.transpose();
	}
	covariance /= (num_points - 1);

	// 3. 特征分解
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
	if (eigen_solver.info() != Eigen::Success)
		return false;

	// 4. 取最小特征值对应向量
	_n = eigen_solver.eigenvectors().col(0);

	return true;
}


NSP_SLAM_LYJ_MATH_END