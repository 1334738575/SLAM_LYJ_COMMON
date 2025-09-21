#ifndef SLAM_LYJ_COMMON_FLANNSEARCH_H
#define SLAM_LYJ_COMMON_FLANNSEARCH_H


#include <flann/flann.hpp>
#include <vector>
#include <iostream>
#include <memory>
#include <chrono>

// �ڴ�ӳ���ļ�֧�֣����������ݣ�
#ifdef USE_BOOST_MMAP
#include <boost/iostreams/device/mapped_file.hpp>
#endif

namespace SLAM_LYJ
{

    template<typename T, int DIM>
    class FLANNWrapper {
    public:

        using Clock = std::chrono::high_resolution_clock;
        // �ڴ��Ż���ʹ�������ڴ��
        std::vector<T> dataset;
        T* dataPtr = nullptr;
        size_t data_size = 0;

        // ��������ָ��
        std::shared_ptr<flann::Index<flann::L2<T>>> index;

        // �����������Զ������Ż���
        void build_index(T* data, size_t rows, bool use_mmap = false) {
            //auto t_start = Clock::now();

            // �ڴ�������
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
    //            // ʹ���ڴ�ӳ���ļ�����������
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

        // ������ѯ�Ż���OpenMP���У�
        template <size_t K>
        void batch_search(const T* queries, size_t num_queries,
            std::vector<int>& indices, std::vector<T>& dists) {
            flann::Matrix<T> query_mat(const_cast<T*>(queries), num_queries, DIM);
            flann::Matrix<int> idx_mat(indices.data(), num_queries, K);
            flann::Matrix<T> dist_mat(dists.data(), num_queries, K);

            // ������������
            flann::SearchParams params;
            params.cores = std::thread::hardware_concurrency(); // �Զ������߳���
            params.use_heap = flann::FLANN_True;  // �Ѽ���
            params.sorted = flann::FLANN_True;    // �������

            index->knnSearch(query_mat, idx_mat, dist_mat, K, params);
        }
    };


}


#endif // !SLAM_LYJ_COMMON_FLANNSEARCH_H
