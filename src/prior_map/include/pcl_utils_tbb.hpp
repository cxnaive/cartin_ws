#ifndef PCL_UTILS_TBB_HPP
#define PCL_UTILS_TBB_HPP

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/tbb.h>

#include <pcl_nanoflann.hpp>
template <typename PointT>
inline void PointCloudInBox_tbb(int num_threads, const pcl::PointCloud<PointT> &cloud,
                                const Eigen::Vector4d &min_pt, const Eigen::Vector4d &max_pt,
                                pcl::PointCloud<PointT> &cloud_out) {
    if (cloud.empty()) {
        cloud_out = pcl::PointCloud<PointT>();
        return;
    }
    tbb::concurrent_vector<std::shared_ptr<pcl::PointCloud<PointT>>> part_clouds;
    auto filter_func = [&](const tbb::blocked_range<size_t> &r) {
        auto part_cloud = std::make_shared<pcl::PointCloud<PointT>>();
        // std::cout << "test: " << r.end() - r.begin() << std::endl;
        for (size_t i = r.begin(); i != r.end(); ++i) {
            if (!std::isfinite(cloud[i].x) || !std::isfinite(cloud[i].y) ||
                !std::isfinite(cloud[i].z))
                continue;
            if (cloud[i].x < min_pt[0] || cloud[i].y < min_pt[1] || cloud[i].z < min_pt[2])
                continue;
            if (cloud[i].x > max_pt[0] || cloud[i].y > max_pt[1] || cloud[i].z > max_pt[2])
                continue;
            part_cloud->points.push_back(cloud[i]);
        }
        part_clouds.emplace_back(std::move(part_cloud));
    };
    int iteration_size = cloud.size();
    int grain_size = std::min(std::max(1, iteration_size / num_threads), iteration_size - 1);
    tbb::task_arena arena(num_threads);
    arena.execute([&] {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, iteration_size, grain_size), filter_func,
                          tbb::simple_partitioner());
        // tbb::parallel_for(tbb::blocked_range<size_t>(0, iteration_size), filter_func);
    });

    cloud_out = *part_clouds[0];
    for (size_t i = 1; i < part_clouds.size(); ++i) {
        cloud_out += *part_clouds[i];
    }
}

template <typename PointT>
inline void PointIndicesInBox_tbb(int num_threads, pcl::PointCloud<PointT> &cloud,
                                  const Eigen::Vector4d &min_pt, const Eigen::Vector4d &max_pt,
                                  pcl::Indices &indices) {
    if (cloud.empty()) return;
    indices = pcl::Indices();
    tbb::concurrent_vector<int> con_indices;
    auto filter_func = [&](const tbb::blocked_range<size_t> &r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            if (!std::isfinite(cloud[i].x) || !std::isfinite(cloud[i].y) ||
                !std::isfinite(cloud[i].z))
                continue;
            if (cloud[i].x < min_pt[0] || cloud[i].y < min_pt[1] || cloud[i].z < min_pt[2])
                continue;
            if (cloud[i].x > max_pt[0] || cloud[i].y > max_pt[1] || cloud[i].z > max_pt[2])
                continue;
            con_indices.push_back(i);
        }
    };
    indices.insert(indices.begin(), con_indices.begin(), con_indices.end());
}

template <typename PointT>
inline void DistanceFilter_tbb(int num_threads, const pcl::PointCloud<PointT> &cloud,
                               const typename PCLAdaptor<PointT>::KDTree &kdtree,
                               double distance_threshold, pcl::PointCloud<PointT> &cloud_out) {
    if (cloud.empty()) {
        cloud_out = pcl::PointCloud<PointT>();
        return;
    }
    tbb::concurrent_vector<std::shared_ptr<pcl::PointCloud<PointT>>> part_clouds;

    auto filter_func = [&](const tbb::blocked_range<size_t> &r) {
        auto part_cloud = std::make_shared<pcl::PointCloud<PointT>>();
        size_t ret_index;
        float sqr_dis;
        nanoflann::KNNResultSet<float> resultSet(1);
        
        for (size_t i = r.begin(); i != r.end(); ++i) {
            if (!std::isfinite(cloud[i].x) || !std::isfinite(cloud[i].y) ||
                !std::isfinite(cloud[i].z))
                continue;

            float query_pt[3] = {cloud[i].x, cloud[i].y, cloud[i].z};
            resultSet.init(&ret_index, &sqr_dis);
            kdtree.findNeighbors(resultSet, query_pt);
            // kdtree.nearestKSearch(cloud[i], 5, indices, distances);
            // float min_distance = *std::min_element(distances.begin(), distances.end());
            // std::cout << "test: " << indices[0] << " " << min_distance << std::endl;
            if (sqr_dis > distance_threshold * distance_threshold)
                part_cloud->points.push_back(cloud[i]);
        }
        part_clouds.emplace_back(std::move(part_cloud));
    };
    int iteration_size = cloud.size();
    int grain_size = std::min(std::max(1, iteration_size / num_threads), iteration_size - 1);
    tbb::task_arena arena(num_threads);
    arena.execute([&] {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, iteration_size, grain_size), filter_func,
                          tbb::simple_partitioner());
        // tbb::parallel_for(tbb::blocked_range<size_t>(0, iteration_size), filter_func);
    });
    cloud_out = *part_clouds[0];
    for (size_t i = 1; i < part_clouds.size(); ++i) {
        cloud_out += *part_clouds[i];
    }
}

#endif