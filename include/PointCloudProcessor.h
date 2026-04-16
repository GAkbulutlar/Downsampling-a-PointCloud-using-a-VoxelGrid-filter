#pragma once

#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudProcessor {
public:
    PointCloudProcessor(float leaf_size_x = 0.01f,
                        float leaf_size_y = 0.01f,
                        float leaf_size_z = 0.01f);

    bool loadPointCloud(const std::string& input_file);
    void printInputSummary() const;
    void estimateNormals(int k_search = 20);
    void downsample();
    void applyTransform(float theta_z, float tx, float ty, float tz);
    void printFilteredSummary() const;
    bool saveFilteredPointCloud(const std::string& output_file) const;

    const pcl::PCLPointCloud2& inputCloud() const;
    const pcl::PCLPointCloud2& filteredCloud() const;
    const pcl::PointCloud<pcl::PointXYZ>& normalXyzCopy() const;
    const pcl::PointCloud<pcl::Normal>& normals() const;

private:
    float leaf_size_x_;
    float leaf_size_y_;
    float leaf_size_z_;
    pcl::PCLPointCloud2::Ptr input_cloud_;
    pcl::PCLPointCloud2::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr normal_xyz_copy_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_;
};