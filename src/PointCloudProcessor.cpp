#include "PointCloudProcessor.h"

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

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
      filtered_cloud_(new pcl::PCLPointCloud2())
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

void PointCloudProcessor::downsample()
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(input_cloud_);
    voxel_grid.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
    voxel_grid.filter(*filtered_cloud_);
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

const pcl::PCLPointCloud2& PointCloudProcessor::filteredCloud() const
{
    return *filtered_cloud_;
}