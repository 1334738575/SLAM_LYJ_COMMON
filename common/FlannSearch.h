#ifndef SLAM_LYJ_COMMON_FLANNSEARCH_H
#define SLAM_LYJ_COMMON_FLANNSEARCH_H


#include <flann/flann.hpp>
#include <vector>
#include <iostream>
#include <memory>
#include <chrono>

// 内存映射文件支持（处理超大数据）
#ifdef USE_BOOST_MMAP
#include <boost/iostreams/device/mapped_file.hpp>
#endif

namespace COMMON_LYJ
{

    template<typename T, int DIM>
    class FLANNWrapper {
    public:

        using Clock = std::chrono::high_resolution_clock;
        // 内存优化：使用连续内存块
        std::vector<T> dataset;
        T* dataPtr = nullptr;
        size_t data_size = 0;

        // 索引智能指针
        std::shared_ptr<flann::Index<flann::L2<T>>> index;

        // 构建索引（自动参数优化）
        void build_index(T* data, size_t rows, bool use_mmap = false) {
            //auto t_start = Clock::now();

            // 内存管理策略
            if (use_mmap) {
                dataset.resize(rows * DIM, 0);
                std::memcpy(dataset.data(), data, rows * DIM * sizeof(T));
            }
            else
            {
                dataPtr = data;
            }
    //        else {
    //#ifdef USE_BOOST_MMAP
    //            // 使用内存映射文件处理超大数据
    //            boost::iostreams::mapped_file_source mmap(
    //                "data.bin", rows * DIM * sizeof(float));
    //            dataset = reinterpret_cast<float*>(mmap.data());
    //#else
    //            throw std::runtime_error("Boost.Mmap not enabled");
    //#endif
    //        }

            flann::Matrix<T> mat_data(dataPtr, rows, DIM);

            index = std::make_shared<flann::Index<flann::L2<T>>>(mat_data, flann::KDTreeIndexParams(4));
            index->buildIndex();

            //std::cout << "Index built in "
            //    << std::chrono::duration_cast<std::chrono::milliseconds>(
            //        Clock::now() - t_start).count() << " ms\n";
        }

        // 批量查询优化（OpenMP并行）
        template <size_t K>
        void batch_search(const T* queries, size_t num_queries,
            std::vector<int>& indices, std::vector<T>& dists) {
            flann::Matrix<T> query_mat(const_cast<T*>(queries), num_queries, DIM);
            flann::Matrix<int> idx_mat(indices.data(), num_queries, K);
            flann::Matrix<T> dist_mat(dists.data(), num_queries, K);

            // 并行搜索参数
            flann::SearchParams params;
            params.cores = std::thread::hardware_concurrency(); // 自动设置线程数
            params.use_heap = flann::FLANN_True;  // 堆加速
            params.sorted = flann::FLANN_True;    // 结果排序

            index->knnSearch(query_mat, idx_mat, dist_mat, K, params);
        }
    };


}


#endif // !SLAM_LYJ_COMMON_FLANNSEARCH_H
