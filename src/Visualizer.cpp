#include "Visualizer.h"

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

void Visualizer::showComparison(const pcl::PCLPointCloud2& original_cloud,
                                const pcl::PCLPointCloud2& filtered_cloud) const
{
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));

    int original_viewport = 0;
    int filtered_viewport = 0;
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, original_viewport);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, filtered_viewport);

    viewer->setBackgroundColor(0.0, 0.0, 0.0, original_viewport);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, filtered_viewport);
    viewer->addText("Original", 10, 10, "original_label", original_viewport);
    viewer->addText("Filtered", 10, 10, "filtered_label", filtered_viewport);

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromPCLPointCloud2(original_cloud, *original_xyz);
    pcl::fromPCLPointCloud2(filtered_cloud, *filtered_xyz);

    viewer->addPointCloud<pcl::PointXYZ>(original_xyz, "original_cloud", original_viewport);
    viewer->addPointCloud<pcl::PointXYZ>(filtered_xyz, "filtered_cloud", filtered_viewport);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        2,
        "original_cloud",
        original_viewport);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        2,
        "filtered_cloud",
        filtered_viewport);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}