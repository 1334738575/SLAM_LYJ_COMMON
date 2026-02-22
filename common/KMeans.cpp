#include "KMeans.h"
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>

namespace COMMON_LYJ
{
    double KMeans::distance(const std::vector<double>& a, const std::vector<double>& b)
    {
        double dist = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
        {
            double diff = a[i] - b[i];
            dist += diff * diff;
        }
        return dist;
    }

    void KMeans::fit(std::vector<std::vector<double>>& data)
    {
        if (data.empty() || k <= 0)
            return;
        n_features = data[0].size();

        // 1. 初始化中心点 (随机选择数据点)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(data.begin(), data.end(), gen); //TODO
        centroids.assign(data.begin(), data.begin() + k);

        // 2. 迭代优化
        for (int iter = 0; iter < max_iter; ++iter)
        {
            // 分配样本到最近中心
            std::vector<std::vector<std::vector<double>>> clusters(k);
            for (const auto& point : data)
            {
                double min_dist = std::numeric_limits<double>::max();
                int cluster_idx = 0;
                for (size_t i = 0; i < centroids.size(); ++i)
                {
                    double dist = distance(point, centroids[i]);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        cluster_idx = i;
                    }
                }
                clusters[cluster_idx].push_back(point);
            }

            // 3. 计算新中心点
            std::vector<std::vector<double>> new_centroids;
            for (const auto& cluster : clusters)
            {
                if (cluster.empty())
                { // 处理空簇
                    new_centroids.push_back(data[gen() % data.size()]);
                    continue;
                }
                std::vector<double> mean(n_features, 0.0);
                for (const auto& pt : cluster)
                {
                    for (int j = 0; j < n_features; ++j)
                    {
                        mean[j] += pt[j];
                    }
                }
                for (auto& val : mean)
                {
                    val /= cluster.size();
                }
                new_centroids.push_back(mean);
            }

            // 4. 收敛检测
            bool converged = true;
            for (size_t i = 0; i < centroids.size(); ++i)
            {
                if (distance(centroids[i], new_centroids[i]) > 1e-6)
                {
                    converged = false;
                    break;
                }
            }
            if (converged)
                break;
            centroids = new_centroids;
        }
    }

    std::vector<int> KMeans::predict(const std::vector<std::vector<double>>& data)
    {
        std::vector<int> labels;
        for (const auto& point : data)
        {
            double min_dist = std::numeric_limits<double>::max();
            int label = -1;
            for (size_t i = 0; i < centroids.size(); ++i)
            {
                double dist = distance(point, centroids[i]);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    label = i;
                }
            }
            labels.push_back(label);
        }
        return labels;
    }

    std::vector<std::vector<double>> KMeans::get_centroids() const
    {
        return centroids;
    }

}