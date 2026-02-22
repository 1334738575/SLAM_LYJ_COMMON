#include "BaseTriMesh.h"

namespace COMMON_LYJ
{
	BaseTriMesh::BaseTriMesh()
	{
	}
	BaseTriMesh::~BaseTriMesh()
	{
	}

	std::vector<Eigen::Vector3f>& BaseTriMesh::getFNormals()
	{
		// TODO: 在此处插入 return 语句
		if (!m_enableFNr)
			std::cout << "face normal is disable." << std::endl;
		return m_fNormals;
	}

	const std::vector<Eigen::Vector3f>& BaseTriMesh::getFNormals() const
	{
		// TODO: 在此处插入 return 语句
		if (!m_enableFNr)
			std::cout << "face normal is disable." << std::endl;
		return m_fNormals;
	}

	const Eigen::Vector3f& BaseTriMesh::getFNormal(uint32_t _id) const
	{
		// TODO: 在此处插入 return 语句
		if (!m_enableFNr)
			std::cout << "face normal is disable." << std::endl;
		return m_fNormals[_id];
	}

	bool BaseTriMesh::setFNormals(const std::vector<Eigen::Vector3f>& _fNormals)
	{
		if (!m_enableFNr)
		{
			std::cout << "face normal is disable." << std::endl;
			return false;
		}
		m_fNormals = _fNormals;
		return true;
	}

	bool BaseTriMesh::setFNormal(uint32_t _id, const Eigen::Vector3f& _fNormal)
	{
		if (!m_enableFNr)
		{
			std::cout << "face normal is disable." << std::endl;
			return false;
		}
		m_fNormals[_id] = _fNormal;
		return true;
	}

	void BaseTriMesh::calculateFNormals()
	{
		if (!m_enableFNr)
			return;
		uint32_t fSize = m_faces.size();
		m_fNormals.resize(fSize);
		Eigen::Vector3f e1, e2;
		for (size_t i = 0; i < fSize; ++i)
		{
			e1 = m_vertexs[m_faces[i].vId_[1]] - m_vertexs[m_faces[i].vId_[0]];
			e2 = m_vertexs[m_faces[i].vId_[2]] - m_vertexs[m_faces[i].vId_[0]];
			m_fNormals[i] = e1.cross(e2);
			if (!m_fNormals[i].isZero())
				m_fNormals[i].normalize();
		}
	}

	void BaseTriMesh::setFCenters(const std::vector<Eigen::Vector3f>& _fCenters)
	{
		if (!m_hasFCtr)
		{
			std::cout << "face center is disable." << std::endl;
			return;
		}
		m_centers = _fCenters;
	}

	void BaseTriMesh::setFCenter(uint32_t _id, const Eigen::Vector3f& _fCenter)
	{
		if (!m_hasFCtr)
		{
			std::cout << "face center is disable." << std::endl;
			return;
		}
		m_centers[_id] = _fCenter;
	}

	std::vector<Eigen::Vector3f>& BaseTriMesh::getFCenters()
	{
		// TODO: 在此处插入 return 语句
		if (!m_hasFCtr)
		{
			std::cout << "face center is disable." << std::endl;
		}
		return m_centers;
	}

	const std::vector<Eigen::Vector3f>& BaseTriMesh::getFCenters() const
	{
		// TODO: 在此处插入 return 语句
		if (!m_hasFCtr)
		{
			std::cout << "face center is disable." << std::endl;
		}
		return m_centers;
	}

	const Eigen::Vector3f& BaseTriMesh::getFCenter(uint32_t _id) const
	{
		// TODO: 在此处插入 return 语句
		if (!m_hasFCtr)
			std::cout << "face center is disable." << std::endl;
		return m_centers[_id];
	}

	void BaseTriMesh::calculateFCenters()
	{
		if (!m_hasFCtr)
		{
			std::cout << "face center is disable." << std::endl;
			return;
		}
		uint32_t fSize = m_faces.size();
		m_centers.resize(fSize);
		for (int i = 0; i < fSize; ++i)
		{
			m_centers[i].setZero();
			for (int j = 0; j < 3; ++j)
			{
				m_centers[i] += m_vertexs[m_faces[i].vId_[j]];
			}
			m_centers[i] /= 3;
		}
		return;
	}

	bool BaseTriMesh::calculateVNormals()
	{
		if (!m_enableFNr)
			return false;
		uint32_t vSize = m_vertexs.size();
		uint32_t fSize = m_faces.size();
		m_vNormals.resize(vSize);
		memset(m_vNormals[0].data(), 0, vSize * 3 * sizeof(float));
		std::vector<int> cnts(vSize, 0);
		for (int i = 0; i < fSize; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				m_vNormals[m_faces[i].vId_[j]] += m_fNormals[i];
				++cnts[m_faces[i].vId_[j]];
			}
		}
		for (size_t i = 0; i < vSize; ++i)
			if (cnts[i])
			{
				// m_vNormals[i] /= cnts[i];
				m_vNormals[i].normalize();
			}
		return true;
	}

	const int BaseTriMesh::getFn() const
	{
		return m_faces.size();
	}

	void BaseTriMesh::subBaseTriMeshByFaces(const std::vector<uint32_t>& _fIds, BaseTriMesh& _outMesh) const
	{
		_outMesh.reset();
		std::unordered_map<uint32_t, uint32_t> vIdOld2New;
		std::vector<Eigen::Vector3f> newVertexs;
		std::vector<BaseTriFace> newFaces;
		newFaces.reserve(_fIds.size());
		for (const auto& fid : _fIds)
		{
			BaseTriFace newFace;
			for (int j = 0; j < 3; ++j)
			{
				uint32_t oldVid = m_faces[fid].vId_[j];
				if (vIdOld2New.find(oldVid) == vIdOld2New.end())
				{
					vIdOld2New[oldVid] = newVertexs.size();
					newVertexs.push_back(m_vertexs[oldVid]);
				}
				newFace.vId_[j] = vIdOld2New[oldVid];
			}
			newFaces.push_back(newFace);
		}
		_outMesh.setVertexs(newVertexs);
		_outMesh.setFaces(newFaces);
		if (isEnableVNormals())
		{
			std::vector<Eigen::Vector3f> newVNormals;
			newVNormals.resize(newVertexs.size());
			for (const auto& vidPair : vIdOld2New)
			{
				newVNormals[vidPair.second] = m_vNormals[vidPair.first];
			}
			_outMesh.enableVNormals();
			_outMesh.setVNormals(newVNormals);
		}
		if (isEnableFNormals())
		{
			std::vector<Eigen::Vector3f> newFNormals;
			newFNormals.resize(newFaces.size());
			for (size_t i = 0; i < _fIds.size(); ++i)
			{
				newFNormals[i] = m_fNormals[_fIds[i]];
			}
			_outMesh.enableFNormals();
			_outMesh.setFNormals(newFNormals);
		}
		if (hasFCenters())
		{
			std::vector<Eigen::Vector3f> newFCenters;
			newFCenters.resize(newFaces.size());
			for (size_t i = 0; i < _fIds.size(); ++i)
			{
				newFCenters[i] = m_centers[_fIds[i]];
			}
			_outMesh.enableFCenters();
			_outMesh.setFCenters(newFCenters);
		}
	}

	bool BaseTriMesh::hasTexture() const
	{
		return m_hasTexture;
	}

	void BaseTriMesh::enableTexture()
	{
		m_hasTexture = true;
		//后续添加默认纹理
	}

	void BaseTriMesh::disableTexture()
	{
		m_hasTexture = false;
		m_textureCoords = std::vector<Eigen::Vector2f>();
		m_texture = COMMON_LYJ::CompressedImage();
	}

	void BaseTriMesh::setTexture(const std::vector<Eigen::Vector2f>& _textureCoords, const std::vector<BaseTriTextureUV>& _triUVs, const COMMON_LYJ::CompressedImage& _texture)
	{
		m_hasTexture = true;
		m_textureCoords = _textureCoords;
		m_triUVs = _triUVs;
		m_texture = _texture;
	}

	std::vector<Eigen::Vector2f>& BaseTriMesh::getTextureCoords()
	{
		// TODO: 在此处插入 return 语句
		return m_textureCoords;
	}

	const std::vector<Eigen::Vector2f>& BaseTriMesh::getTextureCoords() const
	{
		// TODO: 在此处插入 return 语句
		return m_textureCoords;
	}

	COMMON_LYJ::CompressedImage& BaseTriMesh::getTexture()
	{
		// TODO: 在此处插入 return 语句
		return m_texture;
	}

	const COMMON_LYJ::CompressedImage& BaseTriMesh::getTexture() const
	{
		// TODO: 在此处插入 return 语句
		return m_texture;
	}

	std::vector<BaseTriTextureUV>& BaseTriMesh::getTriUVs()
	{
		// TODO: 在此处插入 return 语句
		return m_triUVs;
	}

	const std::vector<BaseTriTextureUV>& BaseTriMesh::getTriUVs() const
	{
		// TODO: 在此处插入 return 语句
		return m_triUVs;
	}

	void BaseTriMesh::reset()
	{
		m_vertexs.clear();
		m_faces.clear();
		m_enableVNr = false;
		m_vNormals.clear();
		m_enableVClr = false;
		m_vColors.clear();
		m_enableFNr = false;
		m_fNormals.clear();
		m_hasFCtr = false;
		m_centers.clear();
		m_hasTexture = false;
		m_textureCoords.clear();
	}

	void BaseTriMesh::transform(const Eigen::Matrix3d& _R, const Eigen::Vector3d& _t)
	{
		if (isEnableFNormals())
		{
			for (int i = 0; i < getFn(); ++i)
			{
				m_fNormals[i] = (_R * m_fNormals[i].cast<double>()).cast<float>();
			}
		}
		Cloud::transform(_R, _t);
	}

}