#include "Visualizer.h"

#include <string>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

void Visualizer::showComparison(const pcl::PCLPointCloud2& original_cloud,
                                const pcl::PCLPointCloud2& downsampled_cloud,
                                const pcl::PCLPointCloud2& rotated_downsampled_cloud,
                                const pcl::PointCloud<pcl::PointXYZ>& normal_xyz_copy,
                                const pcl::PointCloud<pcl::Normal>& knn_normals,
                                int k_search,
                                const pcl::PointCloud<pcl::Normal>& radius_normals,
                                float radius_search) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(original_cloud, *original_xyz);
    pcl::fromPCLPointCloud2(downsampled_cloud, *downsampled_xyz);
    pcl::fromPCLPointCloud2(rotated_downsampled_cloud, *rotated_xyz);

    pcl::PointCloud<pcl::PointXYZ>::Ptr normal_copy_ptr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(normal_xyz_copy);

    // Visualization 1: original vs downsampled.
    pcl::visualization::PCLVisualizer::Ptr viewer1(
        new pcl::visualization::PCLVisualizer("1) Original vs Downsampled"));
    int v1_left  = 0;
    int v1_right = 0;
    viewer1->createViewPort(0.0, 0.0, 0.5, 1.0, v1_left);
    viewer1->createViewPort(0.5, 0.0, 1.0, 1.0, v1_right);
    viewer1->setBackgroundColor(0.0, 0.0, 0.0, v1_left);
    viewer1->setBackgroundColor(0.05, 0.05, 0.1, v1_right);
    viewer1->addText("Original Point Cloud", 10, 10, "v1_left_label", v1_left);
    viewer1->addText("Downsampled Point Cloud", 10, 10, "v1_right_label", v1_right);
    viewer1->addPointCloud<pcl::PointXYZ>(original_xyz, "v1_original", v1_left);
    viewer1->addPointCloud<pcl::PointXYZ>(downsampled_xyz, "v1_downsampled", v1_right);
    viewer1->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "v1_original", v1_left);
    viewer1->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "v1_downsampled", v1_right);

    // Visualization 2: downsampled vs rotated downsampled.
    pcl::visualization::PCLVisualizer::Ptr viewer2(
        new pcl::visualization::PCLVisualizer("2) Downsampled vs Rotated Downsampled"));
    int v2_left  = 0;
    int v2_right = 0;
    viewer2->createViewPort(0.0, 0.0, 0.5, 1.0, v2_left);
    viewer2->createViewPort(0.5, 0.0, 1.0, 1.0, v2_right);
    viewer2->setBackgroundColor(0.0, 0.0, 0.0, v2_left);
    viewer2->setBackgroundColor(0.0, 0.08, 0.08, v2_right);
    viewer2->addText("Downsampled Point Cloud", 10, 10, "v2_left_label", v2_left);
    viewer2->addText("Rotated Downsampled Cloud", 10, 10, "v2_right_label", v2_right);
    viewer2->addPointCloud<pcl::PointXYZ>(downsampled_xyz, "v2_downsampled", v2_left);
    viewer2->addPointCloud<pcl::PointXYZ>(rotated_xyz, "v2_rotated", v2_right);
    viewer2->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "v2_downsampled", v2_left);
    viewer2->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "v2_rotated", v2_right);

    // Visualization 3: kNN normals vs radius-search normals.
    pcl::visualization::PCLVisualizer::Ptr viewer3(
        new pcl::visualization::PCLVisualizer("3) kNN Normals vs Radius Normals"));
    int v3_left  = 0;
    int v3_right = 0;
    viewer3->createViewPort(0.0, 0.0, 0.5, 1.0, v3_left);
    viewer3->createViewPort(0.5, 0.0, 1.0, 1.0, v3_right);
    viewer3->setBackgroundColor(0.0, 0.0, 0.0, v3_left);
    viewer3->setBackgroundColor(0.1, 0.03, 0.03, v3_right);
    viewer3->addText("Downsampled + kNN Normals", 10, 10, "v3_left_label", v3_left);
    viewer3->addText("Downsampled + Radius Normals", 10, 10, "v3_right_label", v3_right);
    viewer3->addText("k = " + std::to_string(k_search), 10, 35, "v3_k_label", v3_left);
    viewer3->addText("r = " + std::to_string(radius_search), 10, 35, "v3_r_label", v3_right);
    viewer3->addPointCloud<pcl::PointXYZ>(normal_copy_ptr, "v3_knn_cloud", v3_left);
    viewer3->addPointCloud<pcl::PointXYZ>(normal_copy_ptr, "v3_radius_cloud", v3_right);
    viewer3->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "v3_knn_cloud", v3_left);
    viewer3->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "v3_radius_cloud", v3_right);

    if (knn_normals.size() > 0 && knn_normals.size() == normal_copy_ptr->size()) {
        pcl::PointCloud<pcl::Normal>::ConstPtr knn_normals_ptr =
            std::make_shared<const pcl::PointCloud<pcl::Normal>>(knn_normals);
        viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
            normal_copy_ptr, knn_normals_ptr,
            /*level=*/10, /*scale=*/0.02f,
            "v3_knn_normals", v3_left);
        viewer3->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "v3_knn_normals");
    }

    if (radius_normals.size() > 0 && radius_normals.size() == normal_copy_ptr->size()) {
        pcl::PointCloud<pcl::Normal>::ConstPtr radius_normals_ptr =
            std::make_shared<const pcl::PointCloud<pcl::Normal>>(radius_normals);
        viewer3->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
            normal_copy_ptr, radius_normals_ptr,
            /*level=*/10, /*scale=*/0.02f,
            "v3_radius_normals", v3_right);
        viewer3->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "v3_radius_normals");
    }

    while (!viewer1->wasStopped() || !viewer2->wasStopped() || !viewer3->wasStopped()) {
        if (!viewer1->wasStopped()) {
            viewer1->spinOnce(50);
        }
        if (!viewer2->wasStopped()) {
            viewer2->spinOnce(50);
        }
        if (!viewer3->wasStopped()) {
            viewer3->spinOnce(50);
        }
    }
}