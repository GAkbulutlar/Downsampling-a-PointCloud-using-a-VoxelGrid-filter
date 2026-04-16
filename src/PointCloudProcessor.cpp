#include "PointCloudProcessor.h"

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace {

void printCloudSummary(const pcl::PCLPointCloud2& cloud, const char* label)
{
    std::cerr << label << ": "
              << cloud.width * cloud.height
              << " data points (" << pcl::getFieldsList(cloud) << ").\n";
}

} // namespace

PointCloudProcessor::PointCloudProcessor(float leaf_size_x,
                                         float leaf_size_y,
                                         float leaf_size_z)
    : leaf_size_x_(leaf_size_x),
      leaf_size_y_(leaf_size_y),
      leaf_size_z_(leaf_size_z),
      input_cloud_(new pcl::PCLPointCloud2()),
      filtered_cloud_(new pcl::PCLPointCloud2()),
      normal_xyz_copy_(new pcl::PointCloud<pcl::PointXYZ>()),
      normals_cloud_(new pcl::PointCloud<pcl::Normal>())
{
}

bool PointCloudProcessor::loadPointCloud(const std::string& input_file)
{
    pcl::PCDReader reader;
    if (reader.read(input_file, *input_cloud_) == -1) {
        std::cerr << "Error: Couldn't read file " << input_file << "\n";
        return false;
    }

    return true;
}

void PointCloudProcessor::printInputSummary() const
{
    printCloudSummary(*input_cloud_, "PointCloud before filtering");
}

void PointCloudProcessor::estimateNormals(int k_search)
{
    // Deep-copy the downsampled cloud into a dedicated XYZ cloud for normal estimation
    normal_xyz_copy_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(*filtered_cloud_, *normal_xyz_copy_);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(normal_xyz_copy_);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    if (k_search <= 0) {
        k_search = 20;
    }
    ne.setKSearch(k_search);
    ne.compute(*normals_cloud_);

    std::cerr << "Normal estimation: " << normals_cloud_->size()
              << " normals computed on downsampled cloud (k=" << k_search << ")\n";
}

void PointCloudProcessor::downsample()
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(input_cloud_);
    voxel_grid.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
    voxel_grid.filter(*filtered_cloud_);
}

void PointCloudProcessor::applyTransform(float theta_z, float tx, float ty, float tz)
{
    // Convert downsampled PCLPointCloud2 to PointXYZ for transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(*filtered_cloud_, *cloud_xyz);

    // Build the affine transform: rotation around Z axis + XYZ translation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << tx, ty, tz;
    transform.rotate(Eigen::AngleAxisf(theta_z, Eigen::Vector3f::UnitZ()));

    std::cerr << "Applying transform (theta_z=" << theta_z
              << " rad, t=[" << tx << ", " << ty << ", " << tz << "])\n";
    std::cerr << transform.matrix() << "\n";

    // Apply transformation and store result back into filtered_cloud_
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_xyz, *transformed_xyz, transform);
    pcl::toPCLPointCloud2(*transformed_xyz, *filtered_cloud_);
}

void PointCloudProcessor::printFilteredSummary() const
{
    printCloudSummary(*filtered_cloud_, "PointCloud after filtering");
}

bool PointCloudProcessor::saveFilteredPointCloud(const std::string& output_file) const
{
    pcl::PCDWriter writer;
    return writer.write(output_file,
                        *filtered_cloud_,
                        Eigen::Vector4f::Zero(),
                        Eigen::Quaternionf::Identity(),
                        false) == 0;
}

const pcl::PCLPointCloud2& PointCloudProcessor::inputCloud() const
{
    return *input_cloud_;
}

const pcl::PointCloud<pcl::PointXYZ>& PointCloudProcessor::normalXyzCopy() const
{
    return *normal_xyz_copy_;
}

const pcl::PointCloud<pcl::Normal>& PointCloudProcessor::normals() const
{
    return *normals_cloud_;
}

const pcl::PCLPointCloud2& PointCloudProcessor::filteredCloud() const
{
    return *filtered_cloud_;
}