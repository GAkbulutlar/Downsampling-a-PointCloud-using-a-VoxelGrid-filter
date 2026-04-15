#include "PointCloudProcessor.h"
#include "Visualizer.h"

namespace {

constexpr char kInputFile[] = "table_scene_lms400.pcd";
constexpr char kOutputFile[] = "table_scene_lms400_downsampled.pcd";

} // namespace

int main()
{
    PointCloudProcessor processor;
    if (!processor.loadPointCloud(kInputFile)) {
        return -1;
    }

    processor.printInputSummary();
    processor.downsample();
    processor.printFilteredSummary();

    if (!processor.saveFilteredPointCloud(kOutputFile)) {
        return -1;
    }

    Visualizer visualizer;
    visualizer.showComparison(processor.inputCloud(), processor.filteredCloud());

    return 0;
}