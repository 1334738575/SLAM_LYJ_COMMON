#ifndef COMMON_LYJ_POSSION_SOLVER_H
#define COMMON_LYJ_POSSION_SOLVER_H


#include "base/Base.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Sparse>


namespace COMMON_LYJ
{
	template<typename TL, int DIML, int DIMX>
	class PossionSolver
	{
	public:
		using EigenL = Eigen::Matrix<TL, DIML, 1>;

		PossionSolver() {
			Bs_.resize(DIMX);
			Xs_.resize(DIMX);
		};
		~PossionSolver() {};

		//void setBoundaries(const std::vector<EigenL>& _boundaries)
		//{
		//	boundaries_ = _boundaries;
		//}
		//void setInners(const std::vector<EigenL>& _inners)
		//{
		//	inners_ = _inners;
		//	int innerSize = inners_.size();
		//	A_.resize(innerSize, innerSize);
		//	for (int i = 0; i < DIMX; ++i)
		//	{
		//		Bs_[i].resize(innerSize);
		//		Bs_[i].setZero();
		//		Xs_[i].resize(innerSize);
		//		Xs_[i].setZero();
		//	}
		//}
		virtual void getDDX(const EigenL& _l, std::vector<Eigen::Triplet<double>>& _vs, bool _updateVs, Eigen::Matrix<double, DIMX, 1>& _ddX) = 0;
		bool solve()
		{
			int sss = Xs_[0].rows();
			A_.resize(sss, sss);
			A_.setFromTriplets(vs_.begin(), vs_.end());
			//Eigen::MatrixXd Aden = A_.toDense();
			//for (int i = 0; i < Aden.rows(); ++i)
			//{
			//	for (int j = 0; j < Aden.cols(); ++j)
			//	{
			//		if (Aden(i, j) != Aden(j, i))
			//			std::cout << " 111" << std::endl;
			//	}
			//}
			Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
			Eigen::FullPivLU<Eigen::MatrixXd> solver2;
			solver.compute(A_);
			if (solver.info() != Eigen::Success) {
				std::cout << "failed!" << std::endl;
				return false;
			}
			for (int i = 0; i < DIMX; ++i)
			{
				Xs_[i] = solver.solve(Bs_[i]);
				if (solver.info() != Eigen::Success) {
					std::cout << "failed2!" << std::endl;
					return false;
				}
			}
			return true;
		}

	protected:
		std::vector<Eigen::VectorXd> Bs_;
		Eigen::SparseMatrix<double> A_;
		std::vector<Eigen::Triplet<double>> vs_;
		std::vector<Eigen::VectorXd> Xs_;
		//std::vector<EigenL> boundaries_;
		//std::vector<EigenL> inners_;
	};

	class SLAM_LYJ_API ImagePossionSolver : public PossionSolver<int, 2, 3>
	{
	public:
		ImagePossionSolver();
		~ImagePossionSolver();

		void possionSolve(cv::Mat& _src, cv::Mat& _tgt, cv::Mat& _mask, cv::Point2i& _center);

		// Í¨¹ý PossionSolver ¼Ì³Ð
		void getDDX(const Eigen::Vector2i& _l, std::vector<Eigen::Triplet<double>>& _vs, bool _updateVs, Eigen::Vector3d& _ddX) override;

	private:
		cv::Mat* src_ = nullptr;
		cv::Mat* tgt_ = nullptr;
		cv::Mat* mask_ = nullptr;
		cv::Point2i center_;
		std::unordered_map<int, int> loc2Inds_;
		//cv::Mat srcLap_;
		//cv::Mat tgtLap_;
	};


}




#endif // !COMMON_LYJ_POSSION_SOLVER_H
