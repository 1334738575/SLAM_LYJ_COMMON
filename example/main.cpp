#include <iostream>
#include <fstream>
#include <cmath> // 用于sin/cos生成示例数据

#include <config/config.h>

#include <base/Base.h>
#include <base/CameraModule.h>
#include <base/Pose.h>
#include <base/PreDefine.h>
#include <base/Triangler.h>

#include <common/Cloud.h>
#include <common/Grid.h>
#include <common/Diffuser.h>
#include <common/BaseTriMesh.h>
#include <common/KdTree.h>
#include <common/Line.h>
#include <common/ThreadPool.h>
#include <common/SVDICP.h>
#include <common/KMeans.h>
#include <common/CommonAlgorithm.h>
#include <common/PossionSolver.h>
#include <common/FlannSearch.h>

#include <IO/MeshIO.h>
#include <IO/SimpleIO.h>
#include <IO/BaseIO.h>

#include <nlohmann/json.hpp>

#include <vector>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void testDefine()
{

    std::cout << SLAM_LYJ_HOME_PATH << std::endl;

    SLAM_LYJ::LYJBuffer buffer;
    SLAM_LYJ::CameraType camType = SLAM_LYJ::CameraType::FISHEYE;
    SLAM_LYJ::PinholeCamera cam(100, 100, std::vector<double>{1, 1, 1, 1});
    SLAM_LYJ::Pose3D pose3D;
    std::cout << LYJOPT->sysHomePath << std::endl;
    SLAM_LYJ::TrianglerPoint3DOption triOpt;
    SLAM_LYJ::TrianglerLine3D triangler;

    SLAM_LYJ::SLAM_LYJ_MATH::Cloud cld;
    SLAM_LYJ::SLAM_LYJ_MATH::Grid<float, 2> grid;
    SLAM_LYJ::SLAM_LYJ_MATH::Diffuser2D diffuser(100, 100);
    SLAM_LYJ::SLAM_LYJ_MATH::BaseTriMesh btm;
    SLAM_LYJ::SLAM_LYJ_MATH::KdTree2d kdTree;
    SLAM_LYJ::SLAM_LYJ_MATH::KdTree<float, 3> kdTree3D;
    SLAM_LYJ::SLAM_LYJ_MATH::Line2d line2d;
    SLAM_LYJ::SLAM_LYJ_MATH::Line3<float> line3f;
    SLAM_LYJ::SLAM_LYJ_MATH::ThreadPool threadPool(4);
    SLAM_LYJ::SLAM_LYJ_MATH::svd_icp(
        {Eigen::Vector3f(1, 2, 3), Eigen::Vector3f(4, 5, 6)},
        {Eigen::Vector3f(7, 8, 9), Eigen::Vector3f(10, 11, 12)});
    SLAM_LYJ::SLAM_LYJ_MATH::KMeans kmeans(3, 100);
    SLAM_LYJ::SLAM_LYJ_MATH::BitFlagVec btf(100);
    auto ret = SLAM_LYJ::SLAM_LYJ_MATH::factorial(5, 1);
}

void adjustPose()
{
    using namespace SLAM_LYJ;
    using namespace SLAM_LYJ::SLAM_LYJ_MATH;
    Eigen::Vector3f o(0, 0, 0);
    Eigen::Vector3f x(1, 0, 0);
    Eigen::Vector3f y(0, 1, 0);
    Eigen::Vector3f z(0, 0, 1);
    std::vector<Eigen::Vector3f> ps;
    ps.push_back(o);
    ps.push_back(x);
    ps.push_back(y);
    ps.push_back(z);
    std::vector<Eigen::Vector3f> clrs = ps;
    for (int i = 0; i < clrs.size(); ++i)
    {
        clrs[i] *= 255;
    }
    BaseTriMesh btm;
    btm.setVertexs(ps);
    btm.enableVColors();
    btm.setVColors(clrs);
    SLAM_LYJ::writePLY("D:/tmp/baseCoord.ply", btm);

    Pose3D Tbt;
    Eigen::Vector3d t(1, -1, 0);
    Eigen::Vector3d tny = -1 * t;
    tny.normalize();
    Tbt.getR() = Rodrigues2RotMatrix<double>(y.cast<double>(), tny);
    Tbt.gett() = t;
    Eigen::Vector3d tx = Tbt.getR().col(0);
    Eigen::Vector3d ty = Tbt.getR().col(1);
    Eigen::Vector3d tz = Tbt.getR().col(2);
    std::vector<Eigen::Vector3f> pts;
    pts.push_back(t.cast<float>());
    pts.push_back(pts[0] + tx.cast<float>());
    pts.push_back(pts[0] + ty.cast<float>());
    pts.push_back(pts[0] + tz.cast<float>());
    BaseTriMesh btmT;
    btmT.setVertexs(pts);
    btmT.enableVColors();
    btmT.setVColors(clrs);
    SLAM_LYJ::writePLY("D:/tmp/tCoord.ply", btmT);

    Pose3D Tbc;
    Eigen::Vector3d c(1, 2, 3);
    Eigen::Vector3d cny = -1 * c;
    cny.normalize();
    Tbc.getR() = Rodrigues2RotMatrix<double>(y.cast<double>(), cny);
    Tbc.gett() = c;
    Eigen::Vector3d cx = Tbc.getR().col(0);
    Eigen::Vector3d cy = Tbc.getR().col(1);
    Eigen::Vector3d cz = Tbc.getR().col(2);
    std::vector<Eigen::Vector3f> pcs;
    pcs.push_back(c.cast<float>());
    pcs.push_back(pcs[0] + cx.cast<float>());
    pcs.push_back(pcs[0] + cy.cast<float>());
    pcs.push_back(pcs[0] + cz.cast<float>());
    BaseTriMesh btmC;
    btmC.setVertexs(pcs);
    btmC.enableVColors();
    btmC.setVColors(clrs);
    SLAM_LYJ::writePLY("D:/tmp/cCoord.ply", btmC);

    if (c(1) < 0)
    {
        if (cz(1) > 0)
        {
            Eigen::Matrix3d Rbc2 = Rodrigues2RotMatrix<double>(cy, PI) * Tbc.getR();
            Tbc.setR(Rbc2);
            Eigen::Vector3d c2x = Tbc.getR().col(0);
            Eigen::Vector3d c2y = Tbc.getR().col(1);
            Eigen::Vector3d c2z = Tbc.getR().col(2);
            std::vector<Eigen::Vector3f> pc2s;
            pc2s.push_back(c.cast<float>());
            pc2s.push_back(pc2s[0] + c2x.cast<float>());
            pc2s.push_back(pc2s[0] + c2y.cast<float>());
            pc2s.push_back(pc2s[0] + c2z.cast<float>());
            BaseTriMesh btmC2;
            btmC2.setVertexs(pc2s);
            btmC2.enableVColors();
            btmC2.setVColors(clrs);
            SLAM_LYJ::writePLY("D:/tmp/cCoord2.ply", btmC2);
        }
    }
    if (c(1) > 0)
    {
        if (cz(1) > 0)
        {
            Eigen::Matrix3d Rbc2 = Rodrigues2RotMatrix<double>(cy, PI) * Tbc.getR();
            Tbc.setR(Rbc2);
            Eigen::Vector3d c2x = Tbc.getR().col(0);
            Eigen::Vector3d c2y = Tbc.getR().col(1);
            Eigen::Vector3d c2z = Tbc.getR().col(2);
            std::vector<Eigen::Vector3f> pc2s;
            pc2s.push_back(c.cast<float>());
            pc2s.push_back(pc2s[0] + c2x.cast<float>());
            pc2s.push_back(pc2s[0] + c2y.cast<float>());
            pc2s.push_back(pc2s[0] + c2z.cast<float>());
            BaseTriMesh btmC2;
            btmC2.setVertexs(pc2s);
            btmC2.enableVColors();
            btmC2.setVColors(clrs);
            SLAM_LYJ::writePLY("D:/tmp/cCoord2.ply", btmC2);
        }
    }
}

// 生成简单四面体模型
void write_obj_file(const std::string &filename)
{
    std::ofstream objFile(filename);
    if (!objFile.is_open())
    {
        throw std::runtime_error("无法创建文件: " + filename);
    }

    // 头部注释 (包含生成时间)
    objFile << "# Generated by C++ on Sun Jul 27 2025\n";
    objFile << "# Vertex Count: 4\n";
    objFile << "# Face Count: 4\n\n";

    // 定义顶点 (单位立方体内)
    const std::vector<std::vector<float>> vertices = {
        {0.0f, 0.0f, 0.0f}, // v1
        {1.0f, 0.0f, 0.0f}, // v2
        {0.5f, 1.0f, 0.0f}, // v3
        {0.5f, 0.5f, 1.0f}  // v4 (顶部)
    };

    // 写入顶点
    for (const auto &v : vertices)
    {
        objFile << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    }

    // 定义面 (四面体)
    const std::vector<std::vector<int>> faces = {
        {1, 2, 3}, // 底面三角形
        {1, 2, 4}, // 前侧面
        {2, 3, 4}, // 右侧面
        {3, 1, 4}  // 左侧面
    };

    // 写入面 (注意OBJ索引从1开始)
    objFile << "\n# Face definitions\n";
    for (const auto &f : faces)
    {
        objFile << "f " << f[0] << " " << f[1] << " " << f[2] << "\n";
    }

    objFile.close();
}

int testOBJ()
{
    try
    {
        write_obj_file("D:/tmp/output.obj");
        std::cout << "OBJ文件生成成功！" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "错误: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int testPLY()
{
    SLAM_LYJ::SLAM_LYJ_MATH::BaseTriMesh btmIn;
    SLAM_LYJ::readPLY("D:/tmp/res_mesh5.ply", btmIn);
    SLAM_LYJ::writePLYBin("D:/tmp/copy2.ply", btmIn);
    return 0;

    // 创建立方体顶点（边长2，中心在原点）
    std::vector<Eigen::Vector3f> vertices = {
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1}, {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}};
    std::vector<Eigen::Vector3f> vColors = {
        {0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0, 1, 1}, {1, 0, 1}, {1, 1, 0}, {1, 1, 1}};
    // 创建立方体面（三角形）
    std::vector<SLAM_LYJ::SLAM_LYJ_MATH::BaseTriFace> faces = {
        // 底面
        {0, 1, 2},
        {2, 3, 0},
        // 顶面
        {4, 5, 6},
        {6, 7, 4},
        // 前面
        {0, 3, 7},
        {7, 4, 0},
        // 后面
        {1, 2, 6},
        {6, 5, 1},
        // 左面
        {0, 4, 5},
        {5, 1, 0},
        // 右面
        {3, 2, 6},
        {6, 7, 3}};

    SLAM_LYJ::SLAM_LYJ_MATH::BaseTriMesh btm;
    btm.setVertexs(vertices);
    btm.enableVColors();
    btm.setVColors(vColors);
    btm.setFaces(faces);
    SLAM_LYJ::writePLY("D:/tmp/cube.ply", btm);
    SLAM_LYJ::writePLYBin("D:/tmp/cubeBin.ply", btm);
    return 0;
}

void testIO()
{
    {
        std::vector<Eigen::Vector3f> ps = {
            Eigen::Vector3f(1, 2, 3),
            Eigen::Vector3f(4, 5, 6),
            Eigen::Vector3f(7, 8, 9)};
        std::ofstream fw("D:/tmp/testIO.bin");
        SLAM_LYJ::writeBinaryVector(fw, ps);
        fw.close();
    }

    {
        std::vector<Eigen::Vector3f> ps;
        std::ifstream fr("D:/tmp/testIO.bin");
        SLAM_LYJ::readBinaryVector(fr, ps);
        fr.close();
    }
}

// 示例用法
int main2()
{
    SLAM_LYJ::BaseTriMesh btm;
    SLAM_LYJ::readPLYMesh("D:/tmp/copy3.ply", btm);
    SLAM_LYJ::writePLYMesh("D:/tmp/copy3.ply", btm);

    return 0;
}

//#include <opencv2/opencv.hpp>
//#include <Eigen/Sparse>
//
//using namespace cv;
//using namespace Eigen;
//
//// 自动生成椭圆掩码（基于图像中心）
//Mat createAutoMask(Mat& img) {
//    Mat mask = Mat::zeros(img.size(), CV_8UC1);
//    ellipse(mask, Point(img.cols / 2, img.rows / 2),
//        Size(img.cols / 3, img.rows / 3),
//        0, 0, 360, Scalar(255), -1);
//    return mask;
//}
//
//void smartPoissonBlend(Mat& target, Mat& source) {
//    // 自动对齐到目标中心
//    Point center(target.cols / 2 - source.cols / 2,
//        target.rows / 2 - source.rows / 2);
//
//    // 自动生成掩码
//    Mat mask = createAutoMask(source);
//    //cv::imshow("222", mask);
//    //cv::waitKey();
//
//    // 多通道处理
//    std::vector<Mat> sourceChannels, targetChannels;
//    split(source, sourceChannels);
//    split(target, targetChannels);
//
//    for (int c = 0; c < 3; ++c) {
//        // 构建稀疏系统
//        SparseMatrix<double> A(source.rows * source.cols, source.rows * source.cols);
//        VectorXd b(source.rows * source.cols);
//        b.setZero();
//        std::vector<Triplet<double>> coeffs;
//
//        int idx = 0;
//        for (int y = 0; y < source.rows; ++y) {
//            for (int x = 0; x < source.cols; ++x) {
//                if (mask.at<uchar>(y, x) == 0) {
//                    coeffs.emplace_back(idx, idx, 1);
//                    b[idx] = targetChannels[c].at<uchar>(y + center.y, x + center.x);
//                    idx++;
//                    continue;
//                }
//
//                double sum = 0, rhs = 0;
//                for (int dy = -1; dy <= 1; dy += 2) {
//                    for (int dx = -1; dx <= 1; dx += 2) {
//                        if (x + dx >= 0 && x + dx < source.cols && y + dy >= 0 && y + dy < source.rows) {
//                            sum += 1;
//                            rhs += sourceChannels[c].at<uchar>(y + dy, x + dx) -
//                                sourceChannels[c].at<uchar>(y, x);
//                        }
//                    }
//                }
//                coeffs.emplace_back(idx, idx, sum);
//                b[idx] = rhs;
//                idx++;
//            }
//        }
//
//        A.setFromTriplets(coeffs.begin(), coeffs.end());
//
//        // 求解线性系统
//        ConjugateGradient<SparseMatrix<double>> solver;
//        solver.compute(A);
//        if (solver.info() != Eigen::Success)
//            std::cout << "failed!" << std::endl;
//        VectorXd retx = solver.solve(b);
//        if (solver.info() != Eigen::Success)
//            std::cout << "failed2!" << std::endl;
//
//        // 写入结果
//        idx = 0;
//        cv::Mat retM(source.rows, source.cols, CV_8UC1);
//        retM.setTo(0);
//        for (int y = 0; y < source.rows; ++y) {
//            for (int x = 0; x < source.cols; ++x) {
//                int ccc = retx[idx];
//                retM.at<uchar>(y, x) = (uchar)ccc;
//                if (mask.at<uchar>(y, x) != 0) {
//                    int ty = y + center.y;
//                    int tx = x + center.x;
//                    if (ty >= 0 && ty < target.rows && tx >= 0 && tx < target.cols) {
//                        targetChannels[c].at<uchar>(ty, tx) =
//                            saturate_cast<uchar>(retx[idx]);
//                    }
//                }
//                idx++;
//            }
//        }
//        cv::imshow("ttt", retM);
//        cv::waitKey();
//        std::ofstream fff("D:/tmp/possion/x.txt");
//        fff << retx;
//        fff.close();
//        continue;
//    }
//    merge(targetChannels, target);
//    //cv::imshow("333", target);
//    //cv::waitKey();
//}
//
//int main3() {
//    //Mat dst = imread("D:/tmp/possion/60.png");     // 目标图像
//    //Mat src = imread("D:/tmp/possion/imgPart.png");     // 源图像
//
//    //if (src.empty() || dst.empty()) {
//    //    std::cerr << "Error loading images!" << std::endl;
//    //    return -1;
//    //}
//
//    //// 自动创建掩码（假设源图像非透明区域为前景）
//    //cv::Mat mask;
//    //cv::cvtColor(src, mask, cv::COLOR_BGR2GRAY);
//    //cv::threshold(mask, mask, 1, 255, cv::THRESH_BINARY);
//
//    //// 设置融合位置（默认目标图像中心）
//    //cv::Point center(dst.cols / 2, dst.rows / 2);
//
//    //// 执行Poisson融合
//    //cv::Mat result;
//    //cv::seamlessClone(src, dst, mask, center, result, cv::NORMAL_CLONE);
//
//    //// 显示并保存结果
//    //cv::imshow("Result", result);
//    //cv::imwrite("fusion_result.jpg", result);
//    //cv::waitKey(0);
//    //return 0;
//
//    Mat target = imread("D:/tmp/possion/60.png");     // 目标图像
//    Mat source = imread("D:/tmp/possion/imgPart.png");     // 源图像
//
//    // 自动缩放源图像到目标尺寸的60%
//    double scale = 0.6 * std::min(target.rows, target.cols) /
//        (double)std::max(source.rows, source.cols);
//    resize(source, source, Size(), scale, scale);
//    //cv::imshow("111", source);
//    //cv::waitKey();
//
//    smartPoissonBlend(target, source);
//
//    imwrite("D:/tmp/possion/blended.jpg", target);
//    return 0;
//}


int main3()
{
    long start, end;
    start = clock();

    cv::Mat img = cv::imread("D:/tmp/possion/imgPart2.png");
    cv::Mat img2 = cv::imread("D:/tmp/possion/60.png");
    //cv::pyrDown(img2, img2);
    //cv::pyrDown(img2, img2);
    //cv::pyrDown(img, img);
    //cv::pyrDown(img, img);
    //cv::pyrDown(img, img);
    //cv::pyrDown(img, img);
    cv::Rect rect(0, 0, img.cols, img.rows);
    cv::Mat img1 = img(rect);
    std::cout << img1.rows << " " << img1.cols << std::endl;

    COMMON_LYJ::ImagePossionSolver possionSolver;
    cv::Mat mask;
    cv::Point2i center(1000, 1000);
    possionSolver.possionSolve(img1, img2, mask, center);

    end = clock();
    std::cout << "used time: " << ((double)(end - start)) / CLOCKS_PER_SEC << " second" << std::endl;
    cv::imshow("result", img2);
    cv::waitKey(0);
    return 0;
}


// 使用示例
int main4() {
    size_t DIM = 3;      // 特征维度（例如SIFT特征）
    size_t BATCH_SIZE = 1; // 批处理查询数量

    // 生成10^6个128维随机数据（生产环境应从文件加载）
    const size_t NUM_DATA = 10000;
    std::vector<float> big_data(NUM_DATA * DIM);
    std::generate(big_data.begin(), big_data.end(),
        []() { return static_cast<float>(rand()) / RAND_MAX; });

    SLAM_LYJ::FLANNWrapper<float, 3> flann;

    // 构建并保存索引
    flann.build_index(big_data.data(), NUM_DATA);
    //flann.save_index("large_index.flann");

    // 生成批查询数据
    std::vector<float> queries(BATCH_SIZE * DIM, 0);
    std::vector<int> indices(BATCH_SIZE * 1); // 每个查询返回10个结果
    std::vector<float> dists(BATCH_SIZE * 1);

    // 执行批量查询
    auto t_start = SLAM_LYJ::FLANNWrapper<float, 3>::Clock::now();
    flann.batch_search<1>(queries.data(), BATCH_SIZE, indices, dists);

    std::cout << "Batch search completed in "
        << std::chrono::duration_cast<std::chrono::microseconds>(
            SLAM_LYJ::FLANNWrapper<float, 3>::Clock::now() - t_start).count()
        << " μs\n";

    for (int i = 0; i < 1; ++i)
    {
        std::cout << i << " :" << indices[i] << " " << dists[i] << std::endl;
        std::cout << big_data[indices[i] * DIM] << " " << big_data[indices[i] * DIM + 1] << " " << big_data[indices[i] * DIM + 2] << std::endl;
		float dd = 
			big_data[indices[i] * DIM] * big_data[indices[i] * DIM] +
			big_data[indices[i] * DIM + 1] * big_data[indices[i] * DIM + 1] +
			big_data[indices[i] * DIM + 2] * big_data[indices[i] * DIM + 2];
        std::cout << dd << std::endl;
    }

    return 0;
}

void testJson()
{
    using json = nlohmann::json;
    json jsonData;
    jsonData["name"] = "lyj";
    jsonData["age"] = 27;
    jsonData["job"] = "algorithm engeneer";
    jsonData["skills"] = { "C++", "Python", "C" };
    jsonData["score"]["math"] = 100;
    jsonData["score"]["english"] = 80;
    std::ofstream fJson("D:/tmp/test.json", std::ios::binary);
    fJson << jsonData.dump(4); //美化输出，缩进4个空格
    fJson.close();

    std::ifstream fJsonIn("D:/tmp/test.json", std::ios::binary);
    json jsonDataIn;
    fJsonIn >> jsonDataIn;
    fJsonIn.close();
    std::cout << jsonDataIn.dump(4) << std::endl;
    std::cout << jsonDataIn["name"] << std::endl;
    std::cout << jsonDataIn["age"] << std::endl;
    std::cout << jsonDataIn["skills"] << std::endl;
    std::cout << jsonDataIn["skills"][0] << std::endl;
    std::cout << jsonDataIn["score"] << std::endl;
    std::cout << jsonDataIn["score"]["math"] << std::endl;
    return;
}

void genData()
{
    COMMON_LYJ::drawCube("D:/tmp/cube2.ply", 1, 2, 3);
    // 定义三个相机位姿 [R|t]
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> poses = {
        {   // 正面视角
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix(),
            Eigen::Vector3d(0, 0, -5)
        },
        {   // 右侧视角
            Eigen::AngleAxisd(PI / 8, Eigen::Vector3d::UnitY()).toRotationMatrix(),
            Eigen::Vector3d(-3, 0, -5)
        },
        {   // 俯视视角
            Eigen::AngleAxisd(-PI / 8, Eigen::Vector3d::UnitX()).toRotationMatrix(),
            Eigen::Vector3d(0, -2, -5)
        }
    };
    SLAM_LYJ::Pose3D Twc1(poses[0].first, poses[0].second);
    SLAM_LYJ::Pose3D Twc2(poses[1].first, poses[1].second);
    SLAM_LYJ::Pose3D Twc3(poses[2].first, poses[2].second);
    struct CameraIntrinsics {
        double fx = 500.0, fy = 500.0; // 焦距
        double cx = 320.0, cy = 240.0; // 主点
        int width = 640, height = 480; // 图像尺寸
    };
    CameraIntrinsics intri;
    SLAM_LYJ::PinholeCamera cam(intri.width, intri.height, intri.fx, intri.fy, intri.cx, intri.cy);
    COMMON_LYJ::drawCam("D:/tmp/Twc1.ply", cam, Twc1, 10);
    COMMON_LYJ::drawCam("D:/tmp/Twc2.ply", cam, Twc2, 10);
    COMMON_LYJ::drawCam("D:/tmp/Twc3.ply", cam, Twc3, 10);
}

#define PI 3.14159265358979323846
using namespace Eigen;
//using namespace SLAM_LYJ;
// 定义三维点和二维点结构体
struct Point3D { Vector3d coord; };
struct Point2D { Vector2d coord; };
struct Line3D { int from, to; };  // 边的起点和终点索引
struct Line2D { Point2D p1, p2; };
// 相机内参结构体（简化版）
struct CameraIntrinsics {
    double fx = 500.0, fy = 500.0; // 焦距
    double cx = 320.0, cy = 240.0; // 主点
    int width = 640, height = 480; // 图像尺寸
};
// 生成立方体顶点和边
void generate_cuboid(double _b, double _l, double _w, double _h, std::vector<Point3D>& vertices, std::vector<Line3D>& edges) {
    vertices.resize(8);
    double l = _l / 2.0;
    double w = _w / 2.0;
    double h = _h / 2.0;
    double bias = 0;
    // 立方体顶点（中心在原点）
    vertices[0].coord << -l + _b, -w + _b, -h + _b;
    vertices[1].coord << l + _b, -w + _b, -h + _b;
    vertices[2].coord << l + _b, w + _b, -h + _b;
    vertices[3].coord << -l + _b, w + _b, -h + _b;
    vertices[4].coord << -l + _b, -w + _b, h + _b;
    vertices[5].coord << l + _b, -w + _b, h + _b;
    vertices[6].coord << l + _b, w + _b, h + _b;
    vertices[7].coord << -l + _b, w + _b, h + _b;

    // 定义12条边
    edges = { {0,1}, {1,2}, {2,3}, {3,0},
             {4,5}, {5,6}, {6,7}, {7,4},
             {0,4}, {1,5}, {2,6}, {3,7} };
}
// 将世界坐标点变换到相机坐标系
Vector3d transform_to_camera(const Vector3d& point,
    const Matrix3d& R, const Vector3d& t) {
    return R.transpose() * (point - t);
}
// 投影三维点到图像平面
bool project_point(const Vector3d& p_cam, Point2D& p_pixel,
    const CameraIntrinsics& cam) {
    if (p_cam.z() <= 0) return false; // 深度为负，不可见

    // 透视投影
    double u = (cam.fx * p_cam.x() / p_cam.z()) + cam.cx;
    double v = (cam.fy * p_cam.y() / p_cam.z()) + cam.cy;

    // 检查是否在图像范围内
    if (u < 0 || u >= cam.width || v < 0 || v >= cam.height)
        return false;

    p_pixel.coord << u, v;
    //p_pixel.coord << p_cam.x() / p_cam.z(), p_cam.y() / p_cam.z();
    return true;
}
// 主函数：生成三个位姿下的二维线观测
int genLineData(std::vector<SLAM_LYJ::Pose3D>& _Twcs, std::vector<SLAM_LYJ::Line3d>& _lines, std::vector<std::vector<SLAM_LYJ::Line2d>>& _allObs, double _bT, double _bl, SLAM_LYJ::PinholeCamera& _cam) {
    // 创建边长为2的立方体
    std::vector<Point3D> vertices;
    std::vector<Line3D> edges_3d;
    generate_cuboid(_bl, 1.0, 2.0, 3.0, vertices, edges_3d);

    _lines.resize(edges_3d.size());
    for (int i = 0; i < edges_3d.size(); ++i)
    {
        const Line3D& edge = edges_3d[i];
        const Vector3d& p1 = vertices[edge.from].coord;
        const Vector3d& p2 = vertices[edge.to].coord;
		std::cout << "(" << p1(0) << ", " << p1(1) << ", " << p1(2) << ") -> ("
			<< p2(0) << ", " << p2(1) << ", " << p2(2) << ")\n";
        _lines[i] = SLAM_LYJ::Line3d(p1, p2);
    }

    // 定义三个相机位姿 [R|t] Twc
    std::vector<std::pair<Matrix3d, Vector3d>> poses = {
        {   // 正面视角
            AngleAxisd(0 + _bT, Vector3d::UnitY()).toRotationMatrix(),
            Vector3d(0, 0, -5)
        },
        {   // 右侧视角
            AngleAxisd(PI / 8 + _bT, Vector3d::UnitY()).toRotationMatrix(),
            Vector3d(-3, 0, -5)
        },
        {   // 俯视视角
            AngleAxisd(-PI / 8 - _bT, Vector3d::UnitX()).toRotationMatrix(),
            Vector3d(0, -2, -5)
        }
    };
    _Twcs.resize(poses.size());
    for (int i = 0; i < poses.size(); ++i)
    {
        _Twcs[i].setR(poses[i].first);
        _Twcs[i].sett(poses[i].second);
    }

    CameraIntrinsics cam{ _cam.fx(), _cam.fy(), _cam.cx(), _cam.cy(), _cam.wide(), _cam.height()}; // 使用默认内参

    _allObs.resize(poses.size());
    for (size_t pose_idx = 0; pose_idx < poses.size(); ++pose_idx) {
        auto& [R, t] = poses[pose_idx];
        std::vector<Line2D> visible_lines;

        // 处理每条3D边
        _allObs[pose_idx].resize(edges_3d.size());
        int cnt = 0;
        for (const Line3D& edge : edges_3d) {
            // 变换起点和终点到相机坐标系
            Vector3d p1_cam = transform_to_camera(
                vertices[edge.from].coord, R, t);
            Vector3d p2_cam = transform_to_camera(
                vertices[edge.to].coord, R, t);

            // 投影到图像平面
            Point2D p1_pixel, p2_pixel;
            bool visible1 = project_point(p1_cam, p1_pixel, cam);
            bool visible2 = project_point(p2_cam, p2_pixel, cam);

            // 仅当两端点均可见时才保留该边
            if (visible1 && visible2) {
                visible_lines.push_back({ p1_pixel, p2_pixel });
                _allObs[pose_idx][cnt] = SLAM_LYJ::Line2d(
                    Eigen::Vector2d(p1_pixel.coord.x(), p1_pixel.coord.y()),
                    Eigen::Vector2d(p2_pixel.coord.x(), p2_pixel.coord.y())
                );
            }
            else {
                _allObs[pose_idx][cnt] = SLAM_LYJ::Line2d();
            }
            ++cnt;
        }

        // 输出当前位姿的观测结果
        std::cout << "Pose " << pose_idx + 1 << "观测到 "
            << visible_lines.size() << " 条线段:\n";
        for (const Line2D& line : visible_lines) {
        	std::cout << " 线段: (" << line.p1.coord.x() << ", "
        		<< line.p1.coord.y() << ") -> ("
        		<< line.p2.coord.x() << ", "
        		<< line.p2.coord.y() << ")\n";
        }
    }

    return 0;
}
void testTriLine3D()
{
    double fx = 500;
    double fy = 500;
    double cx = 320;
    double cy = 240;
    int w = 640;
    int h = 480;
    Eigen::Matrix3d K;
    K << fx, 0, cx,
        0, fy, cy,
        0, 0, 1;
    SLAM_LYJ::PinholeCamera cam(w, h, fx, fy, cx, cy);
    Eigen::Matrix3d KK = SLAM_LYJ::SLAM_LYJ_MATH::Line3d::convertK2KK(K);
    std::vector<SLAM_LYJ::Pose3D> tTwcs;
    std::vector<SLAM_LYJ::Line3d> tline3Dws;
    std::vector<std::vector<SLAM_LYJ::Line2d>> allObs;
    genLineData(tTwcs, tline3Dws, allObs, 0, 0, cam);
    std::vector<SLAM_LYJ::Pose3D> Tcws(tTwcs.size());
    for (size_t i = 0; i < tTwcs.size(); i++)
    {
        Tcws[i] = tTwcs[i].inversed();
        //Tcws[i] = tTwcs[i];
        //Tcws[i].inverse();
    }

    for (int i = 0; i < allObs[0].size(); ++i)
    {
        const auto& l2d1 = allObs[0][i];
        const auto& l2d2 = allObs[1][i];
        const auto& l2d3 = allObs[2][i];

        Eigen::Vector3d pl = tline3Dws[i].sp;
        Eigen::Vector3d p2 = tline3Dws[i].ep;
        Eigen::Vector3d dir = tline3Dws[i].dir;
        Eigen::Vector3d Pc1 = Tcws[0] * pl;
        Eigen::Vector3d Pc2 = Tcws[0] * p2;
        Eigen::Vector2d uv1;
        Eigen::Vector2d uv2;
        cam.world2Image(Pc1, uv1);
        cam.world2Image(Pc2, uv2);
        Eigen::Vector2d uv1t = l2d1.ps.block(0, 0, 2, 1);
        Eigen::Vector2d uv2t = l2d1.ps.block(2, 0, 2, 1);
        std::cout << (uv1 - uv1t) << std::endl;
        std::cout << (uv2 - uv2t) << std::endl;

        Eigen::Vector2d sp1 = l2d1.ps.head<2>();
        Eigen::Vector2d ep1 = l2d1.ps.tail<2>();
        Eigen::Vector3d spn1;
        Eigen::Vector3d epn1;
        cam.image2World(sp1, 10.0, spn1);
        cam.image2World(ep1, 10.0, epn1);
        Eigen::Vector3d pn1 = tTwcs[0] * spn1;
        Eigen::Vector3d pn2 = tTwcs[0] * epn1;
        Eigen::Vector3d pn3 = tTwcs[0].gett();
        SLAM_LYJ::Plane3d pln1(pn1, pn2, pn3);
        Eigen::Vector4d plane1 = pln1.params;
        SLAM_LYJ::BaseTriMesh btm1;
        btm1.addVertex(pn1.cast<float>());
        btm1.addVertex(pn2.cast<float>());
        btm1.addVertex(pn3.cast<float>());
        btm1.addFace(SLAM_LYJ::BaseTriFace(0, 1, 2));
        SLAM_LYJ::writePLYMesh("D:/tmp/pln1.ply", btm1);

        Eigen::Vector2d sp2 = l2d2.ps.head<2>();
        Eigen::Vector2d ep2 = l2d2.ps.tail<2>();
        Eigen::Vector3d spn2;
        Eigen::Vector3d epn2;
        cam.image2World(sp2, 10.0, spn2);
        cam.image2World(ep2, 10.0, epn2);
        Eigen::Vector3d pn12 = tTwcs[1] * spn2;
        Eigen::Vector3d pn22 = tTwcs[1] * epn2;
        Eigen::Vector3d pn32 = tTwcs[1].gett();
        SLAM_LYJ::Plane3d pln2(pn12, pn22, pn32);
        Eigen::Vector4d plane2 = pln2.params;
        SLAM_LYJ::BaseTriMesh btm2;
        btm2.addVertex(pn12.cast<float>());
        btm2.addVertex(pn22.cast<float>());
        btm2.addVertex(pn32.cast<float>());
        btm2.addFace(SLAM_LYJ::BaseTriFace(0, 1, 2));
        SLAM_LYJ::writePLYMesh("D:/tmp/pln2.ply", btm2);

        Eigen::Matrix<double, 6, 1> plkw = SLAM_LYJ::Line3d::pipi_plk(plane1, plane2);
        if (plkw.tail<3>().squaredNorm() < 1e-6)
            continue;
        //plkw = SLAM_LYJ::Line3d::linePN_to_plk(pl, dir);
        {
            std::cout << plkw << std::endl << std::endl;
            Eigen::Matrix<double, 6, 1> plkc = SLAM_LYJ::Line3d::plk_to_pose(plkw, Tcws[2].getR(), Tcws[2].gett());
            std::cout << plkc << std::endl << std::endl;
            Eigen::Matrix<double, 6, 1> plkw2 = SLAM_LYJ::Line3d::plk_to_pose(plkc, tTwcs[2].getR(), tTwcs[2].gett());
            std::cout << plkw2 << std::endl << std::endl;
            Eigen::Matrix<double, 6, 1> plkw3 = SLAM_LYJ::Line3d::plk_from_pose(plkc, Tcws[2].getR(), Tcws[2].gett());
            std::cout << plkw3 << std::endl << std::endl;
            SLAM_LYJ::Pose3D iT = Tcws[2] * tTwcs[2];
            std::cout << iT << std::endl << std::endl;
            Eigen::Vector3d l2d = KK * plkc.head<3>();
            const auto& ps = l2d3.ps;
            double l_norm = l2d(0) * l2d(0) + l2d(1) * l2d(1);
            double l_sqrtnorm = sqrt(l_norm);
            double e1 = ps(0) * l2d(0) + ps(1) * l2d(1) + l2d(2);
            double e2 = ps(2) * l2d(0) + ps(3) * l2d(1) + l2d(2);
            std::cout << e1 / l_sqrtnorm << " " << e2 / l_sqrtnorm << std::endl;
            if ((e1 * e1 + e2 * e2) > 1e-10)
            {
                std::cout << "err!" << std::endl;
            }
        }

        {
            Eigen::Matrix<double, 6, 1> plkc = SLAM_LYJ::Line3d::plk_to_pose(plkw, Tcws[0].getR(), Tcws[0].gett());
            Eigen::Vector3d l2d = KK * plkc.head<3>();
            const auto& ps = l2d1.ps;
            double l_norm = l2d(0) * l2d(0) + l2d(1) * l2d(1);
            double l_sqrtnorm = sqrt(l_norm);
            double e1 = ps(0) * l2d(0) + ps(1) * l2d(1) + l2d(2);
            double e2 = ps(2) * l2d(0) + ps(3) * l2d(1) + l2d(2);
            std::cout << e1 / l_sqrtnorm << " " << e2 / l_sqrtnorm << std::endl;
            if ((e1 * e1 + e2 * e2) > 1e-10)
            {
                std::cout << "err!" << std::endl;
            }
        }

        {
            Eigen::Matrix<double, 6, 1> plkc = SLAM_LYJ::Line3d::plk_to_pose(plkw, Tcws[1].getR(), Tcws[1].gett());
            Eigen::Vector3d l2d = KK * plkc.head<3>();
            const auto& ps = l2d2.ps;
            double l_norm = l2d(0) * l2d(0) + l2d(1) * l2d(1);
            double l_sqrtnorm = sqrt(l_norm);
            double e1 = ps(0) * l2d(0) + ps(1) * l2d(1) + l2d(2);
            double e2 = ps(2) * l2d(0) + ps(3) * l2d(1) + l2d(2);
            std::cout << e1 / l_sqrtnorm << " " << e2 / l_sqrtnorm << std::endl;
            if ((e1 * e1 + e2 * e2) > 1e-10)
            {
                std::cout << "err!" << std::endl;
            }
        }

        continue;
    }
    return;
}

void testLine()
{
    Eigen::Vector2d ctr(1000.36, 1256.99);
    Eigen::Vector2d nrl(6, 9);
    nrl.normalize();
    SLAM_LYJ::Line2d line1(ctr, nrl);
    double x = 1209;
    //double y = (-1 * line1.params[0] * x - line1.params[2]) / line1.params[1];
    double y;
    if(!line1.gety(x, y))
        return;
    Eigen::Vector2d p1(x, y + 1);
    Eigen::Vector2d p2(x, y - 1);
    SLAM_LYJ::Line2d line2(p1, p2);
    Eigen::Vector2d pInt;
    SLAM_LYJ::Line2d::interestL2L(line1, line2, pInt);
    std::cout << p1 << std::endl << std::endl;
    std::cout << p2 << std::endl << std::endl;
    std::cout << pInt << std::endl << std::endl;
}





template<typename T, typename... Args>
void testTemplateArgs(const std::string& _file, Args&&... rest)
{
    using namespace std;
    va_list arg_ptr;
    int sum = 0;
    int nArgValue;

}
void testArgs(const int* _file, ...)
{
    using namespace std;
    va_list arg_ptr;
    int sum = 0;
    const int* nArgValue;
    va_start(arg_ptr, _file);
    for (nArgValue = va_arg(arg_ptr, const int*); nArgValue != 0;)
    {
        sum += (*nArgValue);
        nArgValue = va_arg(arg_ptr, const int*);
    }
    va_end(arg_ptr);
    std::cout << sum << std::endl;
}
class MyStruct : public SLAM_LYJ::BaseLYJ
{
public:
    int id = -1;
    std::string name = "null";

    MyStruct(){}
    MyStruct(int _id, std::string _name) :id(_id), name(_name) {}

    // 通过 BaseLYJ 继承
    void write_binary(std::ofstream& os) override
    {
        COMMON_LYJ::writeBinBasic<int>(os, id);
        COMMON_LYJ::writeBinString<std::string>(os, name);
    }
    void read_binary(std::ifstream& os) override
    {
        COMMON_LYJ::readBinBasic<int>(os, id);
        COMMON_LYJ::readBinString<std::string>(os, name);
    }
};
void testMultiArgs()
{
    using namespace COMMON_LYJ;
    // 测试1：非指针类型 → 返回自身类型
    print_type<target_type_t<int>>("int（非指针）的目标类型");          // int
    print_type<target_type_t<const std::string>>("const string（非指针）的目标类型"); // const string
    print_type<target_type_t<double&>>("double&（非指针引用）的目标类型"); // double

    // 测试2：指针类型 → 返回指向类型
    print_type<target_type_t<int*>>("int* 的目标类型");                // int
    print_type<target_type_t<const std::string*>>("const string* 的目标类型"); // const string
    print_type<target_type_t<double*&>>("double*&（指针引用）的目标类型"); // double
    print_type<target_type_t<int**>>("int**（二级指针）的目标类型");     // int*（二级指针指向一级指针）

    // 测试3：字符串指针（重点验证）
    print_type<target_type_t<const char*>>("const char* 的目标类型");   // const char
    print_type<target_type_t<std::string*>>("string* 的目标类型");      // string

    print_type<target_type_t<MyStruct*>>("MyStruct* 的目标类型");      // string
    print_type<target_type_t<MyStruct>>("MyStruct 的目标类型");      // string
    print_type<target_type_t<const MyStruct&>>("const MyStruct& 的目标类型");      // string
    print_type<target_type_t<const std::vector<MyStruct>&>>("const MyStruct& 的目标类型");      // string


    // 测试1：纯 std::vector 类型
    print_vector_info<const std::vector<int>>("T = std::vector<int>");
    // 输出：是 vector，目标类型 int

    // 测试2：带 const/引用的 vector（清理后仍匹配）
    print_vector_info<const std::vector<std::string>&>("T = const std::vector<string>&");
    // 输出：是 vector，目标类型 string

    // 测试3：非 vector 类型（int）
    print_vector_info<int>("T = int");
    // 输出：否 vector，目标类型 int

    // 测试4：非 vector 类型（指针）
    print_vector_info<int*>("T = int*");
    // 输出：否 vector，目标类型 int*

    // 测试5：嵌套 vector（仅识别外层是 vector，元素类型为内层 vector）
    print_vector_info<std::vector<std::vector<double>>>("T = std::vector<vector<double>>");
    // 输出：是 vector，目标类型 std::vector<double>
    return;

    //int x0 = 0;
    //int x1 = 1;
    //int x2 = 2;
    //int x3 = 3;
    //testArgs(&x0, &x1, &x2, &x3, nullptr);

    int it;
    float flt;
    double dbl;
    char* chPtr;
    std::string strg;
    //strg.c_str
    std::cout << typeid(it).name() << std::endl;
    std::cout << typeid(flt).name() << std::endl;
    std::cout << typeid(dbl).name() << std::endl;
    std::cout << typeid(chPtr).name() << std::endl;
    std::cout << typeid(strg).name() << std::endl;
    std::string tpName = typeid(strg).name();
    std::cout << tpName << std::endl;
    if (tpName.find("std::basic_string<") == 0)
        std::cout << "find " << std::endl;
    std::cout << tpName.find("std::basic_string<") << std::endl;
    uchar uch;
    short sht;
    std::cout << typeid(uch).name() << std::endl;
    std::cout << typeid(sht).name() << std::endl;


    MyStruct x{ 1, "x" };
    MyStruct y{ 3, "y" };
    std::string nameX = typeid(x).name();

    std::vector<MyStruct> strs;
    std::cout << typeid(std::vector<MyStruct>::value_type).name() << std::endl;
    std::cout << typeid(strs).name() << std::endl;
    MyStruct* strPtr;
    std::cout << typeid(strPtr).name() << std::endl;
    std::cout << std::is_pointer_v<decltype(strs)> << std::endl;
    //std::cout << typeid(std::pointer_target_type<decltype(strs)>) << std::endl;
    std::cout << std::is_pointer_v<decltype(strPtr)> << std::endl;
    std::vector<MyStruct*> strPtrs;
    std::cout << typeid(strPtrs).name() << std::endl;
    std::cout << typeid(decltype(strPtrs)).name() << std::endl;
    using T1 = std::remove_reference_t<decltype(*strPtr)>;
    std::cout << typeid(T1).name() << std::endl;

    std::cout << typeid(x).raw_name() << std::endl;
    std::cout << nameX << std::endl;
    decltype(x) z{ 1, "z" };
    std::cout << typeid(z).name() << std::endl;
    //auto nX = #x;
    std::cout << varName(x) << std::endl;
    MyStruct xy{ 10, "xy" };
    std::cout << ADJOINT_NAME(x, y).name << std::endl;
}
void testWriteBinFile()
{
    std::string sv = "12341546";
    std::string fileNameStr = "D:/tmp/testStr.bin";
    COMMON_LYJ::writeBinFile<const std::string>(fileNameStr, sv);
    std::string fileNameStrPtr = "D:/tmp/testStrPtr.bin";
    COMMON_LYJ::writeBinFile<const std::string*>(fileNameStrPtr, &sv);
    std::vector<int> va{ 1,2,3,4 };
    std::string fileNameIntVec = "D:/tmp/testIntVec.bin";
    COMMON_LYJ::writeBinFile<const std::vector<int>>(fileNameIntVec, va);
    std::string fileNameIntVecPtr = "D:/tmp/testIntVecPtr.bin";
    COMMON_LYJ::writeBinFile<const std::vector<int>*>(fileNameIntVecPtr, &va);
    int x0 = 0;
    int x1 = 1;
    int x2 = 2;
    std::vector<int*> vb{&x0, &x1, &x2};
    std::string fileNameIntPtrVec = "D:/tmp/testIntPtrVec.bin";
    COMMON_LYJ::writeBinFile<const std::vector<int*>>(fileNameIntPtrVec, vb);
    std::string fileNameIntPtrVecPtr = "D:/tmp/testIntPtrVecPtr.bin";
    COMMON_LYJ::writeBinFile<const std::vector<int*>*>(fileNameIntPtrVecPtr, &vb);
    MyStruct myStrt{ 100, "lyj" };
    std::string fileNameUser = "D:/tmp/testUser.bin";
    COMMON_LYJ::writeBinFile<const MyStruct>(fileNameUser, myStrt);
}
void testReadBinFile()
{
    std::string sv;
    std::string fileNameStr = "D:/tmp/testStr.bin";
    COMMON_LYJ::readBinFile<std::string>(fileNameStr, sv);
    //std::string fileNameStrPtr = "D:/tmp/testStrPtr.bin";
    //COMMON_LYJ::readBinFile<std::string*>(fileNameStrPtr, &sv);
    std::vector<int> va;
    std::string fileNameIntVec = "D:/tmp/testIntVec.bin";
    COMMON_LYJ::readBinFile<std::vector<int>>(fileNameIntVec, va);
    //std::string fileNameIntVecPtr = "D:/tmp/testIntVecPtr.bin";
    //COMMON_LYJ::readBinFile<std::vector<int>*>(fileNameIntVecPtr, &va);
    int x0;
    int x1;
    int x2;
    std::vector<int*> vb{ &x0, &x1, &x2 };
    std::string fileNameIntPtrVec = "D:/tmp/testIntPtrVec.bin";
    COMMON_LYJ::readBinFile<std::vector<int*>>(fileNameIntPtrVec, vb);
    //std::string fileNameIntPtrVecPtr = "D:/tmp/testIntPtrVecPtr.bin";
    //COMMON_LYJ::readBinFile<std::vector<int*>*>(fileNameIntPtrVecPtr, &vb);
    MyStruct myStrt;
    std::string fileNameUser = "D:/tmp/testUser.bin";
    COMMON_LYJ::readBinFile<MyStruct>(fileNameUser, myStrt);
    return;
}

int main(int argc, char *argv[])
{
    std::cout << "Hello COMMON_LYJ" << std::endl;

    testWriteBinFile();
    testReadBinFile();
    //testMultiArgs();
    // adjustPose();
    // testPLY();
    //main2();
    //main3();
    //main4();
    // testIO();
    // testOBJ();
    //testJson();
    //genData();
    //testTriLine3D();
    //testLine();
    return 0;
}