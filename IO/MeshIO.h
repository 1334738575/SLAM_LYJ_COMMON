#ifndef COMMON_LYJ_MESHIO_H
#define COMMON_LYJ_MESHIO_H


#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <vector>
#include "base/Base.h"
#include "common/BaseTriMesh.h"


NSP_SLAM_LYJ_BEGIN

using namespace SLAM_LYJ::SLAM_LYJ_MATH;

// 二进制写入函数
template <typename T>
void writeBinary(std::ofstream& file, const T& value) {
    file.write(reinterpret_cast<const char*>(&value), sizeof(T));
}
template <typename T>
void writeBinaryVector(std::ofstream& file, const std::vector<T>& vec) {
	size_t size = vec.size();
	writeBinary(file, size);
	file.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(T));
}
SLAM_LYJ_API void writePLY(const std::string& filename, const BaseTriMesh& btm);
SLAM_LYJ_API void writePLYBin(const std::string& filename, const BaseTriMesh& btm); //有bug


// 二进制读取函数
template <typename T>
void readBinary(std::ifstream& file, T& value) {
	file.read(reinterpret_cast<char*>(&value), sizeof(T));
}
template <typename T>
void readBinaryVector(std::ifstream& file, std::vector<T>& vec) {
	size_t size = 0;
	readBinary(file, size);
	vec.resize(size);
	file.read(reinterpret_cast<char*>(vec.data()), size * sizeof(T));
}


NSP_SLAM_LYJ_END


#endif // !COMMON_LYJ_MESHIO_H
