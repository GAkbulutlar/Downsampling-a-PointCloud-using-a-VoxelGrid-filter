#include "PointCloudProcessor.h"
#include "Visualizer.h"

#include <cmath>

namespace {

constexpr char kInputFile[] = "table_scene_lms400.pcd";
constexpr char kOutputFile[] = "table_scene_lms400_downsampled.pcd";
constexpr int kNormalKSearch = 20;

// Transformation parameters
constexpr float kRotationZ  = M_PI / 4.0f;  // 45 degrees around Z axis
constexpr float kTranslateX = 0.0f;         // metres along X
constexpr float kTranslateY = 0.0f;
constexpr float kTranslateZ = 0.0f;

} // namespace

int main()
{
    PointCloudProcessor processor;
    if (!processor.loadPointCloud(kInputFile)) {
        return -1;
    }

    processor.printInputSummary();
    processor.downsample();
    processor.estimateNormals(kNormalKSearch);
    processor.applyTransform(kRotationZ, kTranslateX, kTranslateY, kTranslateZ);
    processor.printFilteredSummary();

    if (!processor.saveFilteredPointCloud(kOutputFile)) {
        return -1;
    }

    Visualizer visualizer;
    visualizer.showComparison(processor.inputCloud(), processor.filteredCloud(),
                              processor.normalXyzCopy(), processor.normals(),
                              kNormalKSearch);

    return 0;
}