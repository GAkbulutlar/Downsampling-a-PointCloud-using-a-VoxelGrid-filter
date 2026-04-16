#include "Visualizer.h"

#include <string>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

void Visualizer::showComparison(const pcl::PCLPointCloud2& original_cloud,
                                const pcl::PCLPointCloud2& filtered_cloud,
                                const pcl::PointCloud<pcl::PointXYZ>& normal_xyz_copy,
                                const pcl::PointCloud<pcl::Normal>& normals,
                                int k_search) const
{
    (void)original_cloud;

    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("kNN Normal Estimation"));

    int left_viewport  = 0;
    int right_viewport = 0;
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, left_viewport);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, right_viewport);

    viewer->setBackgroundColor(0.0, 0.0, 0.0, left_viewport);
    viewer->setBackgroundColor(0.05, 0.05, 0.15, right_viewport);
    viewer->addText("Downsampled Cloud (Input)", 10, 10, "left_label", left_viewport);
    viewer->addText("kNN Normals Overlay", 10, 10, "right_label", right_viewport);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(filtered_cloud, *filtered_xyz);

    pcl::PointCloud<pcl::PointXYZ>::Ptr normal_copy_ptr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(normal_xyz_copy);

    // Left: downsampled cloud only
    viewer->addPointCloud<pcl::PointXYZ>(filtered_xyz, "downsampled_left", left_viewport);

    // Right: same cloud with normals overlay
    viewer->addPointCloud<pcl::PointXYZ>(normal_copy_ptr, "normal_cloud_right", right_viewport);

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "downsampled_left", left_viewport);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "normal_cloud_right", right_viewport);

    if (normals.size() > 0 && normals.size() == normal_copy_ptr->size()) {
        pcl::PointCloud<pcl::Normal>::ConstPtr normals_ptr =
            std::make_shared<const pcl::PointCloud<pcl::Normal>>(normals);
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
            normal_copy_ptr, normals_ptr,
            /*level=*/10, /*scale=*/0.02f,
            "estimated_normals", right_viewport);
        // Colour normals red
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "estimated_normals");
    }

    viewer->addText("k = " + std::to_string(k_search), 10, 35, "k_label", right_viewport);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}