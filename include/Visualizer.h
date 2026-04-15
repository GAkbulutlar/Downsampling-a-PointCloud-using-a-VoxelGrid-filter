#pragma once

#include <pcl/PCLPointCloud2.h>

class Visualizer {
public:
    void showComparison(const pcl::PCLPointCloud2& original_cloud,
                        const pcl::PCLPointCloud2& filtered_cloud) const;
};