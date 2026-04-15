#include <iostream>
#include <fmt/core.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>


int main()
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // Fill in the cloud data
    pcl::PCDReader reader;

    if (reader.read("table_scene_lms400.pcd", *cloud) == -1)
    {
        std::cerr << "Error: Couldn't read file table_scene_lms400.pcd\n";
        return -1;
    }

    std::cerr << "PointCloud before filtering: "
              << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ").\n";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: "
              << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").\n";

    pcl::PCDWriter writer;
    writer.write("table_scene_lms400_downsampled.pcd",
                 *cloud_filtered,
                 Eigen::Vector4f::Zero(),
                 Eigen::Quaternionf::Identity(),
                 false);
// 1. Create the visualizer object
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
// 2. Create two viewports (Left for Original, Right for Filtered)
int v1(0);
int v2(0);
viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // Left half
viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2); // Right half

viewer->setBackgroundColor(0, 0, 0, v1);
viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);

// 3. Add the clouds (We need to convert PCLPointCloud2 to PointXYZ for the viewer)
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_filtered(new pcl::PointCloud<pcl::PointXYZ>);

pcl::fromPCLPointCloud2(*cloud, *temp_cloud);
pcl::fromPCLPointCloud2(*cloud_filtered, *temp_filtered);

viewer->addPointCloud<pcl::PointXYZ>(temp_cloud, "original_cloud", v1);
viewer->addPointCloud<pcl::PointXYZ>(temp_filtered, "filtered_cloud", v2);

// 4. Run the visualizer loop
while (!viewer->wasStopped())
{
    viewer->spinOnce(100);
}
    return 0;
}