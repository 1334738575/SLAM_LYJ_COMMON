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

#include <IO/MeshIO.h>

#include <vector>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <cstdint>

void testDefine()
{

    std::cout << SLAM_LYJ_HOME_PATH << std::endl;

    SLAM_LYJ::LYJBuffer buffer;
    SLAM_LYJ::CameraType camType = SLAM_LYJ::CameraType::FISHEYE;
    SLAM_LYJ::PinholeCamera cam(100, 100, std::vector<double>{1, 1, 1, 1});
    SLAM_LYJ::Pose3D pose3D;
    std::cout << LYJOPT->sysHomePath << std::endl;
    SLAM_LYJ::TriangleOption triOpt;
    SLAM_LYJ::Triangler<double> triangler(triOpt);

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


//#include "pch.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "vector"
#include "time.h"

#define elif else if
#define ATD at<double>
#define vector vector<Mat>

using namespace cv;
using namespace std;

//calculate horizontal gradient, img(i,j+1) - img(i,j)
Mat getGradientXp(Mat& img)
{
    int height = img.rows;
    int width = img.cols;
    Mat cat = repeat(img, 1, 2);
    Rect roi = Rect(1, 0, width, height);
    Mat roimat = cat(roi);
    return roimat - img;
}


//calculate vertical gradient, img(i+1,j) - img(i,j)
Mat getGradientYp(Mat& img)
{
    int height = img.rows;
    int width = img.cols;
    Mat cat = repeat(img, 2, 1);
    Rect roi = Rect(0, 1, width, height);
    Mat roimat = cat(roi);
    return roimat - img;
}

//calculate horizontal gradient, img(i,j-1) - img(i,j)
Mat getGradientXn(Mat& img)
{
    int height = img.rows;
    int width = img.cols;
    Mat cat = repeat(img, 1, 2);
    Rect roi = Rect(width - 1, 0, width, height);
    Mat roimat = cat(roi);
    return roimat - img;
}
//calculate vertical gradient, img(i-1,j) - img(i,j)
Mat getGradientYn(Mat& img)
{
    int height = img.rows;
    int width = img.cols;
    Mat cat = repeat(img, 2, 1);
    Rect roi = Rect(0, height - 1, width, height);
    Mat roimat = cat(roi);
    return roimat - img;
}

int getLabel(int i, int j, int height, int width)
{
    return i * width + j;
}

//get Matrix A.
Mat getA(int height, int width)
{
    Mat A = Mat::eye(height * width, height * width, CV_64FC1);
    A *= -4;
    Mat M = Mat::zeros(height, width, CV_64FC1);
    Mat temp = Mat::ones(height, width - 2, CV_64FC1);
    Rect roi = Rect(1, 0, width - 2, height);
    Mat roimat = M(roi);
    temp.copyTo(roimat);
    temp = Mat::ones(height - 2, width, CV_64FC1);
    roi = Rect(0, 1, width, height - 2);
    roimat = M(roi);
    temp.copyTo(roimat);
    temp = Mat::ones(height - 2, width - 2, CV_64FC1);
    temp *= 2;
    roi = Rect(1, 1, width - 2, height - 2);
    roimat = M(roi);
    temp.copyTo(roimat);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int label = getLabel(i, j, height, width);
            if (M.ATD(i, j) == 0) {
                if (i == 0)  A.ATD(getLabel(i + 1, j, height, width), label) = 1;
                elif(i == height - 1)   A.ATD(getLabel(i - 1, j, height, width), label) = 1;
                if (j == 0)  A.ATD(getLabel(i, j + 1, height, width), label) = 1;
                elif(j == width - 1)   A.ATD(getLabel(i, j - 1, height, width), label) = 1;
            }elif(M.ATD(i, j) == 1) {
                if (i == 0) {
                    A.ATD(getLabel(i + 1, j, height, width), label) = 1;
                    A.ATD(getLabel(i, j - 1, height, width), label) = 1;
                    A.ATD(getLabel(i, j + 1, height, width), label) = 1;
                }elif(i == height - 1) {
                    A.ATD(getLabel(i - 1, j, height, width), label) = 1;
                    A.ATD(getLabel(i, j - 1, height, width), label) = 1;
                    A.ATD(getLabel(i, j + 1, height, width), label) = 1;
                }
                if (j == 0) {
                    A.ATD(getLabel(i, j + 1, height, width), label) = 1;
                    A.ATD(getLabel(i - 1, j, height, width), label) = 1;
                    A.ATD(getLabel(i + 1, j, height, width), label) = 1;
                }elif(j == width - 1) {
                    A.ATD(getLabel(i, j - 1, height, width), label) = 1;
                    A.ATD(getLabel(i - 1, j, height, width), label) = 1;
                    A.ATD(getLabel(i + 1, j, height, width), label) = 1;
                }
            }
            else {
                A.ATD(getLabel(i, j - 1, height, width), label) = 1;
                A.ATD(getLabel(i, j + 1, height, width), label) = 1;
                A.ATD(getLabel(i - 1, j, height, width), label) = 1;
                A.ATD(getLabel(i + 1, j, height, width), label) = 1;
            }
        }
    }
    return A;
}

// Get the following Laplacian matrix
// 0  1  0
// 1 -4  1
// 0  1  0
Mat
getLaplacian() {
    Mat laplacian = Mat::zeros(3, 3, CV_64FC1);
    laplacian.ATD(0, 1) = 1.0;
    laplacian.ATD(1, 0) = 1.0;
    laplacian.ATD(1, 2) = 1.0;
    laplacian.ATD(2, 1) = 1.0;
    laplacian.ATD(1, 1) = -4.0;
    return laplacian;
}
// Calculate b
// using convolution.
Mat getB1(Mat& img1, Mat& img2, int posX, int posY, Rect ROI) {
    Mat Lap;
    filter2D(img1, Lap, -1, getLaplacian());
    int roiheight = ROI.height;
    int roiwidth = ROI.width;
    Mat B = Mat::zeros(roiheight * roiwidth, 1, CV_64FC1);
    for (int i = 0; i < roiheight; i++) {
        for (int j = 0; j < roiwidth; j++) {
            double temp = 0.0;
            temp += Lap.ATD(i + ROI.y, j + ROI.x);
            if (i == 0)              temp -= img2.ATD(i - 1 + posY, j + posX);
            if (i == roiheight - 1)  temp -= img2.ATD(i + 1 + posY, j + posX);
            if (j == 0)              temp -= img2.ATD(i + posY, j - 1 + posX);
            if (j == roiwidth - 1)   temp -= img2.ATD(i + posY, j + 1 + posX);
            B.ATD(getLabel(i, j, roiheight, roiwidth), 0) = temp;
        }
    }
    return B;
}

// Calculate b
// using getGradient functions.
Mat getB2(Mat& img1, Mat& img2, int posX, int posY, Rect ROI) {
    Mat grad = getGradientXp(img1) + getGradientYp(img1) + getGradientXn(img1) + getGradientYn(img1);
    int roiheight = ROI.height;
    int roiwidth = ROI.width;
    Mat B = Mat::zeros(roiheight * roiwidth, 1, CV_64FC1);
    for (int i = 0; i < roiheight; i++) {
        for (int j = 0; j < roiwidth; j++) {
            double temp = 0.0;
            temp += grad.ATD(i + ROI.y, j + ROI.x);
            if (i == 0)              temp -= img2.ATD(i - 1 + posY, j + posX);
            if (i == roiheight - 1)  temp -= img2.ATD(i + 1 + posY, j + posX);
            if (j == 0)              temp -= img2.ATD(i + posY, j - 1 + posX);
            if (j == roiwidth - 1)   temp -= img2.ATD(i + posY, j + 1 + posX);
            B.ATD(getLabel(i, j, roiheight, roiwidth), 0) = temp;
        }
    }
    return B;
}

// Solve equation and reshape it back to the right height and width.
Mat getResult(Mat& A, Mat& B, Rect& ROI) {
    Mat result;
    solve(A, B, result);
    result = result.reshape(0, ROI.height);
    return  result;
}

// img1: 3-channel image, we wanna move something in it into img2.
// img2: 3-channel image, dst image.
// ROI: the position and size of the block we want to move in img1.
// posX, posY: where we want to move the block to in img2
Mat
poisson_blending(Mat& img1, Mat& img2, Rect ROI, int posX, int posY) {

    int roiheight = ROI.height;
    int roiwidth = ROI.width;
    Mat A = getA(roiheight, roiwidth);

    // we must do the poisson blending to each channel.
    vector rgb1;
    split(img1, rgb1);
    vector rgb2;
    split(img2, rgb2);

    vector result;
    Mat merged, res, Br, Bg, Bb;
    // For calculating B, you can use either getB1() or getB2()
    Br = getB2(rgb1[0], rgb2[0], posX, posY, ROI);
    //Br = getB2(rgb1[0], rgb2[0], posX, posY, ROI);
    res = getResult(A, Br, ROI);
    result.push_back(res);
    cout << "R channel finished..." << endl;
    Bg = getB2(rgb1[1], rgb2[1], posX, posY, ROI);
    //Bg = getB2(rgb1[1], rgb2[1], posX, posY, ROI);
    res = getResult(A, Bg, ROI);
    result.push_back(res);
    cout << "G channel finished..." << endl;
    Bb = getB2(rgb1[2], rgb2[2], posX, posY, ROI);
    //Bb = getB2(rgb1[2], rgb2[2], posX, posY, ROI);
    res = getResult(A, Bb, ROI);
    result.push_back(res);
    cout << "B channel finished..." << endl;

    // merge the 3 gray images into a 3-channel image 
    merge(result, merged);
    return merged;
}

int main3()
{
    long start, end;
    start = clock();

    Mat img1, img2;
    Mat in1 = imread("D:/tmp/possion/imgPart.png");
    Mat in2 = imread("D:/tmp/possion/60.png");
    cv::pyrDown(in2, in2);
    cv::pyrDown(in2, in2);
    cv::pyrDown(in2, in2);
    cv::pyrDown(in1, in1);
    //imshow("src", in1);
    //imshow("dst", in2);
    in1.convertTo(img1, CV_64FC3);
    in2.convertTo(img2, CV_64FC3);

    int posXinPic2 = 10;
    int posYinPic2 = 10;

    Rect rc = Rect(0, 0, in1.cols, in1.rows);
    Mat result = poisson_blending(img1, img2, rc, posXinPic2, posYinPic2);
    result.convertTo(result, CV_8UC1);
    Rect rc2 = Rect(posXinPic2, posYinPic2, in1.cols, in1.rows);
    Mat roimat = in2(rc2);
    result.copyTo(roimat);

    end = clock();
    cout << "used time: " << ((double)(end - start)) / CLOCKS_PER_SEC << " second" << endl;
    imwrite("result.png", in2);
    imshow("roi", result);
    imshow("result", in2);

    waitKey(0);
    return 0;
}

int main(int argc, char *argv[])
{
    std::cout << "Hello COMMON_LYJ" << std::endl;
    // adjustPose();
    // testPLY();
    //main2();
    main3();
    // testIO();
    // testOBJ();
    return 0;
}