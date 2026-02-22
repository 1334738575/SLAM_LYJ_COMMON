#ifndef SLAM_LYJ_KMEANS_H
#define SLAM_LYJ_KMEANS_H

#include <iostream>
#include <vector>
#include "common/CommonAlgorithm.h"

namespace COMMON_LYJ
{
    class SLAM_LYJ_API KMeans
    {
    private:
        int k;                                      // 聚类数
        int max_iter;                               // 最大迭代次数
        int n_features;                             // 数据维度
        std::vector<std::vector<double>> centroids; // 聚类中心

        // 计算欧氏距离平方（避免开方提升性能）
        double distance(const std::vector<double>& a, const std::vector<double>& b);

    public:
        KMeans(int k_, int max_iter_ = 300) : k(k_), max_iter(max_iter_) {}

        // 训练入口
        void fit(std::vector<std::vector<double>>& data);

        // 预测样本所属聚类
        std::vector<int> predict(const std::vector<std::vector<double>>& data);

        // 获取中心点
        std::vector<std::vector<double>> get_centroids() const;
    };

    // 示例数据生成器
    std::vector<std::vector<double>> generate_sample_data(int samples, int clusters)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> dist(0.0, 2.0);

        std::vector<std::vector<double>> data;
        for (int i = 0; i < clusters; ++i)
        {
            double center_x = 10.0 * i;
            for (int j = 0; j < samples / clusters; ++j)
            {
                data.push_back({ center_x + dist(gen), dist(gen) });
            }
        }
        return data;
    }

    // // 替换随机初始化为概率选择
    // centroids.push_back(data[gen() % data.size()]);
    // for (int i = 1; i < k; ++i) {
    //     std::vector<double> distances;
    //     for (const auto& pt : data) {
    //         double min_dist = std::numeric_limits<double>::max();
    //         for (const auto& c : centroids) {
    //             min_dist = std::min(min_dist, distance(pt, c));
    //         }
    //         distances.push_back(min_dist);
    //     }
    //     std::discrete_distribution<> d(distances.begin(), distances.end());
    //     centroids.push_back(data[d(gen)]);
    // }

    // int main() {
    //     // 生成测试数据（3个高斯分布簇）
    //     auto data = generate_sample_data(300, 3);
    //     // 训练模型
    //     KMeans model(3);
    //     model.fit(data);
    //     // 输出结果
    //     std::cout << "Cluster Centers:\n";
    //     for (const auto& center : model.get_centroids()) {
    //         printf("(%.2f, %.2f)\n", center[0], center[1]);
    //     }
    //     return 0;
    // }


}

#endif