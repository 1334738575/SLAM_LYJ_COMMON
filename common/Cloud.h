#ifndef SLAM_LYJ_CLOUD_H
#define SLAM_LYJ_CLOUD_H

#include "base/Base.h"

namespace COMMON_LYJ
{
	class SLAM_LYJ_API Cloud
	{
	public:
		Cloud();
		~Cloud();

		void setVertexs(const std::vector<Eigen::Vector3f>& _vertexs);
		void setVertex(uint32_t _id, const Eigen::Vector3f& _vertex);
		void addVertex(const Eigen::Vector3f& _vertex);
		void addVertex(const Eigen::Vector3f&& _vertex);
		std::vector<Eigen::Vector3f>& getVertexs();
		const std::vector<Eigen::Vector3f>& getVertexs() const;
		const Eigen::Vector3f& getVertex(uint32_t _id) const;
		const int getVn() const;

		void enableVNormals();
		void disableVNormals();
		bool isEnableVNormals() const;
		std::vector<Eigen::Vector3f>& getVNormals();
		const std::vector<Eigen::Vector3f>& getVNormals() const;
		Eigen::Vector3f& getVNormal(uint32_t _id);
		const Eigen::Vector3f& getVNormal(uint32_t _id) const;
		bool setVNormals(const std::vector<Eigen::Vector3f>& _vNormals);
		bool setVNormal(uint32_t _id, const Eigen::Vector3f& _vNormal);

		void enableVColors();
		void disableVColors();
		bool isEnableVColors() const;
		std::vector<Eigen::Vector3f>& getVColors();
		const std::vector<Eigen::Vector3f>& getVColors() const;
		const Eigen::Vector3f& getVColor(uint32_t _id) const;
		bool setVColors(const std::vector<Eigen::Vector3f>& _vColors);
		bool setVColor(uint32_t _id, const Eigen::Vector3f& _vColor);

		virtual void reset();
		virtual void transform(const Eigen::Matrix3d& _R, const Eigen::Vector3d& _t);

	protected:
		std::vector<Eigen::Vector3f> m_vertexs;
		bool m_enableVNr = false;
		std::vector<Eigen::Vector3f> m_vNormals;
		bool m_enableVClr = false;
		std::vector<Eigen::Vector3f> m_vColors;
	};

	inline void Cloud::setVertexs(const std::vector<Eigen::Vector3f>& _vertexs)
	{
		m_vertexs = _vertexs;
	}

	inline void Cloud::setVertex(uint32_t _id, const Eigen::Vector3f& _vertex)
	{
		m_vertexs[_id] = _vertex;
	}

	inline std::vector<Eigen::Vector3f>& Cloud::getVertexs()
	{
		return m_vertexs;
	}

	inline const std::vector<Eigen::Vector3f>& Cloud::getVertexs() const
	{
		return m_vertexs;
	}

	inline const Eigen::Vector3f& Cloud::getVertex(uint32_t _id) const
	{
		return m_vertexs[_id];
	}

	inline const int Cloud::getVn() const
	{
		return m_vertexs.size();
	}

	inline bool Cloud::isEnableVNormals() const
	{
		return m_enableVNr;
	}

	inline std::vector<Eigen::Vector3f>& Cloud::getVNormals()
	{
		return m_vNormals;
	}

	inline const std::vector<Eigen::Vector3f>& Cloud::getVNormals() const
	{
		return m_vNormals;
	}

	inline Eigen::Vector3f& Cloud::getVNormal(uint32_t _id)
	{
		return m_vNormals[_id];
	}

	inline const Eigen::Vector3f& Cloud::getVNormal(uint32_t _id) const
	{
		return m_vNormals[_id];
	}

}

#endif // !SLAM_LYJ_CLOUD_H
