#ifndef SLAM_LYJ_RANSAC_H
#define SLAM_LYJ_RANSAC_H

#include "base/Base.h"

#include "common/CommonAlgorithm.h"
#include "common/Line.h"

NSP_SLAM_LYJ_MATH_BEGIN

template<typename DATATYPE, typename ERRORTYPE, typename MODULE>
class RANSAC
{
public:
	typedef std::function<bool(const MODULE& mdl, const DATATYPE& data, ERRORTYPE& err)> FuncCalError;
	typedef std::function<bool(const std::vector<const DATATYPE*>& samples, MODULE& mdl)> FuncCalModule;
	RANSAC(const double _inlineRatioTh, const int _minInlineNum,
		const int _maxIterNum = INT32_MAX, const double _dstInlineRatioTh = 0.99, const double _stopRatio = 1.0)
		:m_stopRatio(_stopRatio), m_minInlineNum(_minInlineNum)
	{
		double dstRat = std::clamp(_dstInlineRatioTh, 0.01, 0.99);
		double invInRatN = 1 - std::pow(_inlineRatioTh, _minInlineNum);
		double invDstRat = 1 - dstRat;
		int maxInterNum = static_cast<int>(std::log(invDstRat) / std::log(invInRatN));
		m_maxIterNum = std::min(_maxIterNum, maxInterNum);
	};
	void setCallBack(FuncCalError _funcCalErr, FuncCalModule _funcCalModule) {
		m_funcCalErr = _funcCalErr;
		m_funcCalModule = _funcCalModule;
	}

	~RANSAC() {};

	double run(const std::vector<DATATYPE>& _datas, std::vector<ERRORTYPE>& _errs, std::vector<bool>& _bInlines, MODULE& _mdl) {
		const int dataSize = (int)_datas.size();
		if (dataSize < m_minInlineNum)
			return 0.0;
		double bestRatio = m_stopRatio/2;
		std::vector<std::vector<int>> allSamples(m_maxIterNum, std::vector<int>(m_minInlineNum, -1));
		generateAllSamples(dataSize, m_minInlineNum, allSamples);
		//可以并行化，或者内部并行化
		for (int i = 0; i < m_maxIterNum; ++i) {
			const std::vector<int>& inds = allSamples[i];
			if (inds[0] == -1)
				continue;
			std::vector<ERRORTYPE> errs(dataSize);
			std::vector<bool> bInlines(dataSize, false);
			MODULE mdl;
			double rat = runOnce(_datas, inds, errs, bInlines, mdl);
			if (rat <= bestRatio)
				continue;
			bestRatio = rat;
			_mdl = mdl;
			_errs = errs;
			_bInlines = bInlines;
			if (rat >= m_stopRatio) {
				break;
			}
		}
		return bestRatio;
	}

protected:
	void generateAllSamples(const int _allSize, const int _eveSize, std::vector<std::vector<int>>& _allSamples) {
		uint64_t selSize = selectNum(_allSize, _eveSize);
		if (selSize < (uint64_t)m_maxIterNum) {
			m_maxIterNum = (int)selSize;
			_allSamples.resize(selSize);
			fullSelection(_allSize, _eveSize, _allSamples);
			return;
		}
		randIndexGroup(_allSize, _eveSize, m_maxIterNum, _allSamples);
	}
	double runOnce(const std::vector<DATATYPE>& _datas, const std::vector<int>& _inds, std::vector<ERRORTYPE>& _errs, std::vector<bool>& _bInlines, MODULE& _mdl) {
		std::vector<const DATATYPE*> samples;
		for (const auto& ind : _inds) {
			samples.push_back(&_datas[ind]);
		}
		if (!m_funcCalModule(samples, _mdl)) {
			return 0.0;
		}
		int dataSize = (int)_datas.size();
		int validCnt = 0;
		for (int i = 0; i < dataSize; ++i) {
			_bInlines[i] = m_funcCalErr(_mdl, _datas[i], _errs[i]);
			if(_bInlines[i])
				++validCnt;
		}
		return static_cast<double>(validCnt)/static_cast<double>(dataSize);
	}


	virtual bool calErr(const MODULE& mdl, const DATATYPE& data, ERRORTYPE& err) { return false; };
	virtual bool calMdl(const std::vector<const DATATYPE*>& samples, MODULE& mdl) {return false; };

protected:
	double m_stopRatio = 1;
	int m_maxIterNum = INT32_MAX;
	int m_minInlineNum = INT32_MAX;
	FuncCalError m_funcCalErr = nullptr;
	FuncCalModule m_funcCalModule = nullptr;
};



class RANSACLine3f : public RANSAC<Eigen::Vector3f, float, Line3f>
{
public:
	RANSACLine3f(const double _errTh, const double _lenTh,
		const double _inlineRatioTh, const int _minInlineNum,
		const int _maxIterNum = INT32_MAX, const double _dstInlineRatioTh = 0.99, const double _stopRatio = 1.0);
	~RANSACLine3f() {};

protected:
	bool calErr(const Line3f& _L3D, const Eigen::Vector3f& _P, float& _err);
	bool calMdl(const std::vector<const Eigen::Vector3f*>& _samples, Line3f& _mdl);

protected:
	double m_errTh = 0.01;
	double m_lenTh = 0;
};

template<typename ERRORTYPE, typename MODULE>
class RANSACWithInd
{
public:
	typedef std::function<int(const MODULE& mdl, const std::vector<int>& datas, std::vector<ERRORTYPE>& errs, std::vector<bool>& bInls)> FuncCalErrors;
	typedef std::function<bool(const std::vector<int>& samples, MODULE& mdl)> FuncCalModule;
	RANSACWithInd(const double _preInlineRatio, const int _minNum2Solve,
		const int _maxIterNum = INT32_MAX, const double _dstSampleRatioTh = 0.99, const double _minRatio = 0.6)
		:m_minRatio(_minRatio), m_minNum2Solve(_minNum2Solve)
	{
		double dstRat = std::clamp(_dstSampleRatioTh, 0.01, 0.99);
		double invInRatN = 1 - std::pow(_preInlineRatio, _minNum2Solve);
		double invDstRat = 1 - dstRat;
		int maxInterNum = static_cast<int>(std::log(invDstRat) / std::log(invInRatN));
		m_maxIterNum = std::min(_maxIterNum, maxInterNum);
		m_maxIterNum = std::max(1, m_maxIterNum);
	};
	void setCallBack(FuncCalErrors _funcCalErrs, FuncCalModule _funcCalModule) {
		m_funcCalErrs = _funcCalErrs;
		m_funcCalModule = _funcCalModule;
	}

	~RANSACWithInd() {};

	double run(const int& _dataSize, std::vector<ERRORTYPE>& _errs, std::vector<bool>& _bInlines, MODULE& _mdl, std::vector<int>& _bestSample) {
		std::vector<int> datas(_dataSize);
		int indTmp = 0;
		std::generate(datas.begin(), datas.end(), [&indTmp]{ return indTmp++;});
		return run(datas, _errs, _bInlines, _mdl, _bestSample);
	}
	double run(const std::vector<int>& _datas, std::vector<ERRORTYPE>& _errs, std::vector<bool>& _bInlines, MODULE& _mdl, std::vector<int>& _bestSample) {
		const int dataSize = (int)_datas.size();
		if (dataSize < m_minNum2Solve)
			return 0.0;
		double bestRatio = 0;
		std::vector<std::vector<int>> allSamples(m_maxIterNum, std::vector<int>(m_minNum2Solve, -1));
		generateAllSamples(dataSize, m_minNum2Solve, allSamples);
		//可以并行化，或者内部并行化
		std::vector<ERRORTYPE> errs(dataSize);
		std::vector<bool> bInlines(dataSize, false);
		MODULE mdl;
		int bestI = -1;
		for (int i = 0; i < m_maxIterNum; ++i) {
			const std::vector<int>& inds = allSamples[i]; 
			if (inds[0] == -1)
				continue;
			double rat = runOnce(_datas, inds, errs, bInlines, mdl);
			if(rat < m_minRatio)
				continue;
			if (rat <= bestRatio)
				continue;
			bestRatio = rat;
			_mdl = mdl;
			_errs = errs;
			_bInlines = bInlines;
			bestI = i;
		}
		if(bestI != -1)
			_bestSample = allSamples[bestI];
		return bestRatio;
	}

protected:
	void generateAllSamples(const int _allSize, const int _eveSize, std::vector<std::vector<int>>& _allSamples) {
		randIndexGroup(_allSize, _eveSize, m_maxIterNum, _allSamples);
	}
	double runOnce(const std::vector<int>& _datas, const std::vector<int>& _inds, std::vector<ERRORTYPE>& _errs, std::vector<bool>& _bInlines, MODULE& _mdl) {
		std::vector<int> samples;
		for (const auto& ind : _inds) {
			samples.push_back(_datas[ind]);
		}
		if (!m_funcCalModule(samples, _mdl)) {
			return 0.0;
		}
		int validCnt = m_funcCalErrs(_mdl, _datas, _errs, _bInlines);
		return static_cast<double>(validCnt)/static_cast<double>(_datas.size());
	}

protected:
	double m_minRatio = 1;
	int m_maxIterNum = INT32_MAX;
	int m_minNum2Solve = INT32_MAX;
	FuncCalErrors m_funcCalErrs = nullptr;
	FuncCalModule m_funcCalModule = nullptr;
};



NSP_SLAM_LYJ_MATH_END

#endif //SLAM_LYJ_RANSAC_H