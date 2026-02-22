#ifndef COMMON_LYJ_MESHIO_H
#define COMMON_LYJ_MESHIO_H


#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <vector>
#include "base/Base.h"
#include "common/BaseTriMesh.h"


namespace COMMON_LYJ
{

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
    SLAM_LYJ_API void writePLYBin(const std::string& filename, const BaseTriMesh& btm); //二进制有bug


    // 二进制读取函数
    template <typename T>
    void readBinary(std::ifstream& file, T& value) {
        file.read(reinterpret_cast<char*>(&value), sizeof(T));
    }
    SLAM_LYJ_API void readBinaryFloat(std::ifstream& file, float& value);
    template <typename T>
    void readBinaryVector(std::ifstream& file, std::vector<T>& vec) {
        size_t size = 0;
        readBinary(file, size);
        vec.resize(size);
        file.read(reinterpret_cast<char*>(vec.data()), size * sizeof(T));
    }
    SLAM_LYJ_API void readPLY(const std::string& filename, BaseTriMesh& btm);
    SLAM_LYJ_API void readPLYBin(const std::string& filename, BaseTriMesh& btm); //二进制有bug



    // 顶点数据结构
    struct Vertex {
        float x = 0, y = 0, z = 0;
        float nx = 0, ny = 0, nz = 0;
        uint8_t r = 255, g = 255, b = 255, a = 255;
    };

    // 面片数据结构
    struct Face {
        std::vector<int> indices;
    };

    class SLAM_LYJ_API PLYReader {
    public:
        bool read(const std::string& filename,
            std::vector<Vertex>& vertices,
            std::vector<Face>& faces) {
            std::ifstream file(filename, std::ios::binary);
            if (!file) return false;

            // 解析头部
            HeaderInfo header = parseHeader(file);

            // 读取数据
            readData(file, header, vertices, faces);
            return true;
        }
        bool read(const std::string& filename,
            BaseTriMesh& btm) {
            std::ifstream file(filename, std::ios::binary);
            if (!file) return false;

            // 解析头部
            HeaderInfo header = parseHeader(file);

            // 读取数据
            readPLY(file, header, btm);
            return true;
        }

    private:
        struct Property {
            std::string name;
            std::string type;
            bool isList = false;
            std::string listSizeType;
            std::string listItemType;
        };

        struct Element {
            std::string name;
            size_t count;
            std::vector<Property> properties;
        };

        struct HeaderInfo {
            bool isBinaryLittleEndian = true;
            std::vector<Element> elements;
        };

        HeaderInfo parseHeader(std::istream& file) {
            HeaderInfo header;
            std::string line;

            while (std::getline(file, line) && line != "end_header") {
                std::istringstream iss(line);
                std::string token;
                iss >> token;

                if (token == "format") {
                    std::string format, version;
                    iss >> format >> version;
                    header.isBinaryLittleEndian = (format == "binary_little_endian");
                }
                else if (token == "element") {
                    Element element;
                    iss >> element.name >> element.count;
                    header.elements.push_back(element);
                }
                else if (token == "property") {
                    std::string type, name;
                    if (header.elements.back().name == "face") {
                        // 处理面片列表属性
                        std::string listTag;
                        iss >> listTag >> type >> name;
                        Property prop;
                        prop.isList = true;
                        prop.listSizeType = listTag;
                        prop.listItemType = type;
                        prop.name = name;
                        header.elements.back().properties.push_back(prop);
                    }
                    else {
                        iss >> type >> name;
                        header.elements.back().properties.push_back({ name, type });
                    }
                }
            }
            return header;
        }

        template <typename T>
        T readType(std::istream& file) {
            T value;
            file.read(reinterpret_cast<char*>(&value), sizeof(T));
            if (!file) throw std::runtime_error("读取失败");
            return value;
        }

        template <typename T>
        T readSwap(std::istream& file) {
            T value = readType<T>(file);
            if (!true)
            {
                char* data = reinterpret_cast<char*>(&value);
                std::reverse(data, data + sizeof(T));
            }
            return value;
        }

        void readData(std::istream& file,
            const HeaderInfo& header,
            std::vector<Vertex>& vertices,
            std::vector<Face>& faces) {
            for (const auto& element : header.elements) {
                if (element.name == "vertex") {
                    vertices.resize(element.count);
                    for (size_t i = 0; i < element.count; ++i) {
                        Vertex& v = vertices[i];
                        for (const auto& prop : element.properties) {
                            if (prop.name == "x") v.x = readSwap<float>(file);
                            else if (prop.name == "y") v.y = readSwap<float>(file);
                            else if (prop.name == "z") v.z = readSwap<float>(file);
                            else if (prop.name == "nx") v.nx = readSwap<float>(file);
                            else if (prop.name == "ny") v.ny = readSwap<float>(file);
                            else if (prop.name == "nz") v.nz = readSwap<float>(file);
                            else if (prop.name == "red") v.r = readSwap<uint8_t>(file);
                            else if (prop.name == "green") v.g = readSwap<uint8_t>(file);
                            else if (prop.name == "blue") v.b = readSwap<uint8_t>(file);
                            else if (prop.name == "alpha") v.a = readSwap<uint8_t>(file);
                        }
                    }
                }
                else if (element.name == "face") {
                    faces.resize(element.count);
                    for (size_t i = 0; i < element.count; ++i) {
                        Face& face = faces[i];
                        uint8_t count = readSwap<uint8_t>(file);
                        face.indices.resize(count);
                        for (uint8_t j = 0; j < count; ++j) {
                            face.indices[j] = readSwap<int>(file);
                        }
                    }
                }
            }
        }

        void readPLY(std::istream& file,
            const HeaderInfo& header,
            BaseTriMesh& btm) {
            auto& vertices = btm.getVertexs();
            auto& faces = btm.getFaces();
            for (const auto& element : header.elements) {
                if (element.name == "vertex") {
                    vertices.resize(element.count);
                    bool hasNr = false;
                    bool hasClr = false;
                    for (const auto& prop : element.properties) {
                        if (prop.name == "nx") {
                            hasNr = true;
                            btm.enableVNormals();
                        }
                        else if (prop.name == "red") {
                            hasClr = true;
                            btm.enableVColors();
                        }
                    }
                    auto& vNrs = btm.getVNormals();
                    auto& vClrs = btm.getVColors();
                    uint8_t clr[3];
                    for (size_t i = 0; i < element.count; ++i) {
                        auto& v = vertices[i];
                        for (const auto& prop : element.properties) {
                            if (prop.name == "x") v(0) = readSwap<float>(file);
                            else if (prop.name == "y") v(1) = readSwap<float>(file);
                            else if (prop.name == "z") v(2) = readSwap<float>(file);
                            else if (prop.name == "nx") vNrs[i](0) = readSwap<float>(file);
                            else if (prop.name == "ny") vNrs[i](1) = readSwap<float>(file);
                            else if (prop.name == "nz") vNrs[i](2) = readSwap<float>(file);
                            else if (prop.name == "red") clr[0] = readSwap<uint8_t>(file);
                            else if (prop.name == "green") clr[1] = readSwap<uint8_t>(file);
                            else if (prop.name == "blue") clr[2] = readSwap<uint8_t>(file);
                            else if (prop.name == "alpha") readSwap<uint8_t>(file);
                            if (hasClr) {
                                vClrs[i](0) = clr[0] / 255.0f;
                                vClrs[i](1) = clr[1] / 255.0f;
                                vClrs[i](2) = clr[2] / 255.0f;
                            }
                        }
                    }
                }
                else if (element.name == "face") {
                    faces.resize(element.count);
                    for (size_t i = 0; i < element.count; ++i) {
                        BaseTriFace& face = faces[i];
                        uint8_t count = readSwap<uint8_t>(file);
                        //face.vId_.resize(count);
                        for (uint8_t j = 0; j < count; ++j) {
                            face.vId_[j] = readSwap<int>(file);
                        }
                    }
                }
            }
        }

    };

    // 写入PLY文件
    class SLAM_LYJ_API PLYWriter {
    public:
        void write(const std::string& filename,
            const std::vector<Vertex>& vertices,
            const std::vector<Face>& faces,
            bool writeColors = true) {
            std::ofstream file(filename, std::ios::binary);
            if (!file) throw std::runtime_error("无法创建文件");

            // 写入头部
            file << "ply\n";
            file << "format binary_little_endian 1.0\n";
            file << "element vertex " << vertices.size() << "\n";
            file << "property float x\n";
            file << "property float y\n";
            file << "property float z\n";
            if (!vertices.empty() && (vertices[0].nx != 0 || vertices[0].ny != 0 || vertices[0].nz != 0)) {
                file << "property float nx\n";
                file << "property float ny\n";
                file << "property float nz\n";
            }
            if (writeColors) {
                file << "property uchar red\n";
                file << "property uchar green\n";
                file << "property uchar blue\n";
                file << "property uchar alpha\n";
            }
            file << "element face " << faces.size() << "\n";
            file << "property list uchar int vertex_indices\n";
            file << "end_header\n";

            // 写入顶点数据
            for (const auto& v : vertices) {
                file.write(reinterpret_cast<const char*>(&v.x), sizeof(float) * 3);
                if (!vertices.empty() && (v.nx != 0 || v.ny != 0 || v.nz != 0)) {
                    file.write(reinterpret_cast<const char*>(&v.nx), sizeof(float) * 3);
                }
                if (writeColors) {
                    file.write(reinterpret_cast<const char*>(&v.r), sizeof(uint8_t) * 4);
                }
            }

            // 写入面片数据
            for (const auto& f : faces) {
                uint8_t count = static_cast<uint8_t>(f.indices.size());
                file.write(reinterpret_cast<const char*>(&count), sizeof(uint8_t));
                file.write(reinterpret_cast<const char*>(f.indices.data()), sizeof(int) * count);
            }
        }
        void write(const std::string& filename,
            const BaseTriMesh& btm) {
            std::ofstream file(filename, std::ios::binary);
            if (!file) throw std::runtime_error("无法创建文件");

            const auto& vertices = btm.getVertexs();
            const auto& faces = btm.getFaces();
            const auto& vNrs = btm.getVNormals();
            const auto& vClrs = btm.getVColors();
            bool hasVNr = btm.isEnableVNormals();
            bool hasVClr = btm.isEnableVColors();
            int vSize = btm.getVn();
            int fSize = btm.getFn();

            // 写入头部
            file << "ply\n";
            file << "format binary_little_endian 1.0\n";
            file << "element vertex " << vertices.size() << "\n";
            file << "property float x\n";
            file << "property float y\n";
            file << "property float z\n";
            if (hasVNr) {
                file << "property float nx\n";
                file << "property float ny\n";
                file << "property float nz\n";
            }
            if (hasVClr) {
                file << "property uchar red\n";
                file << "property uchar green\n";
                file << "property uchar blue\n";
                //file << "property uchar alpha\n";
            }
            file << "element face " << faces.size() << "\n";
            file << "property list uchar int vertex_indices\n";
            file << "end_header\n";

            // 写入顶点数据
            uint8_t clr[3];
            for (int i = 0; i < vSize; ++i) {
                file.write(reinterpret_cast<const char*>(vertices[i].data()), sizeof(float) * 3);
                if (hasVNr) {
                    file.write(reinterpret_cast<const char*>(vNrs[i].data()), sizeof(float) * 3);
                }
                if (hasVClr) {
                    clr[0] = (uint8_t)(vClrs[i](0) * 255);
                    clr[1] = (uint8_t)(vClrs[i](1) * 255);
                    clr[2] = (uint8_t)(vClrs[i](2) * 255);
                    file.write(reinterpret_cast<const char*>(clr), sizeof(uint8_t) * 3);
                }
            }

            // 写入面片数据
            for (const auto& f : faces) {
                uint8_t count = 3;
                file.write(reinterpret_cast<const char*>(&count), sizeof(uint8_t));
                file.write(reinterpret_cast<const char*>(f.vId_), sizeof(int) * count);
            }
        }
    };

    SLAM_LYJ_API void writePLYMesh(const std::string& filename, const BaseTriMesh& btm);
    SLAM_LYJ_API void readPLYMesh(const std::string& filename, BaseTriMesh& btm);


    SLAM_LYJ_API bool writeOBJMesh(const std::string& filename, const BaseTriMesh& btm);
    SLAM_LYJ_API bool readOBJMesh(const std::string& filename, BaseTriMesh& btm);

}


#endif // !COMMON_LYJ_MESHIO_H
