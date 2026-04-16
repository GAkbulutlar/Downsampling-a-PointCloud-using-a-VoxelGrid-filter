#pragma once

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Visualizer {
public:
    void showComparison(const pcl::PCLPointCloud2& original_cloud,
                        const pcl::PCLPointCloud2& downsampled_cloud,
                        const pcl::PCLPointCloud2& rotated_downsampled_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>& normal_xyz_copy,
                        const pcl::PointCloud<pcl::Normal>& knn_normals,
                        int k_search,
                        const pcl::PointCloud<pcl::Normal>& radius_normals,
                        float radius_search) const;
};