# Downsampling a PointCloud Using a VoxelGrid Filter

This project is a small C++ Point Cloud Library (PCL) example that demonstrates how to reduce the number of points in a dense point cloud using a `VoxelGrid` filter.

The program:

- loads a `.pcd` point cloud file,
- downsamples it by grouping nearby points into fixed-size 3D voxels,
- writes the filtered result back to disk,
- opens a PCL visualizer with the original cloud on the left and the filtered cloud on the right.

## What the Project Does

The executable defined in [CMakeLists.txt](CMakeLists.txt) is built from [VoxelGridFilter.cpp](VoxelGridFilter.cpp).

At runtime, the code performs these steps:

1. Reads `table_scene_lms400.pcd` into a `pcl::PCLPointCloud2` object.
2. Prints the number of input points and available fields.
3. Applies a `pcl::VoxelGrid` filter with a leaf size of `0.01f x 0.01f x 0.01f`.
4. Prints the number of points after filtering.
5. Saves the result as `table_scene_lms400_downsampled.pcd`.
6. Converts both clouds to `pcl::PointCloud<pcl::PointXYZ>` for visualization.
7. Displays both clouds in a split 3D viewer for direct comparison.

## Result
![Original and Filtered Pointcloud](image.png)  

## Why VoxelGrid Filtering Is Useful

Point clouds captured from LiDAR, RGB-D cameras, or 3D scanners often contain far more points than are needed for many downstream tasks. A `VoxelGrid` filter reduces that density by dividing space into a 3D grid and replacing all points inside each voxel with a representative point.

This gives you a smaller cloud that is usually:

- faster to process,
- easier to visualize,
- more practical for registration, segmentation, and feature extraction pipelines.

The tradeoff is detail: smaller leaf sizes preserve more geometry, while larger leaf sizes produce stronger compression.

## Build Configuration in This Workspace

This workspace is configured as a CMake project using:

- CMake presets,
- the `Ninja` generator,
- the `msvc-vcpkg` configure preset,
- Microsoft C++ compiler (`cl.exe`),
- dependencies installed through `vcpkg`.

The preset is defined in [CMakePresets.json](CMakePresets.json) and writes build output to:

- `build/msvc-vcpkg`

The project links against these libraries:

- `fmt`
- `pcl_common`
- `pcl_io`
- `pcl_filters`
- `pcl_visualization`

## Important Runtime Requirement

The source code loads the input file using a relative path:

- `table_scene_lms400.pcd`

That means the file must be available in the program's current working directory when the executable runs, or the code will exit with:

```text
Error: Couldn't read file table_scene_lms400.pcd
```

The generated output file is:

- `table_scene_lms400_downsampled.pcd`

## How to Build

From the workspace root, a typical preset-based build is:

```powershell
cmake --preset msvc-vcpkg
cmake --build --preset msvc-vcpkg
```

If you are building through VS Code with CMake Tools, use the same preset and build the `MyCppCode` target.

## How to Run

Make sure `table_scene_lms400.pcd` is available where the executable will look for it, then run the built program from the build directory:

```powershell
.\build\msvc-vcpkg\MyCppCode.exe
```

When it succeeds, you should see:

- console output reporting point counts before and after filtering,
- a new downsampled `.pcd` file written to disk,
- a two-panel viewer showing the original cloud and the filtered cloud.

## Source Files

- [MyCppCode.cpp](VoxelGridFilter.cpp): main example implementation.
- [CMakeLists.txt](CMakeLists.txt): target definition and library linkage.
- [CMakePresets.json](CMakePresets.json): preset-based build configuration.

## Notes for This Setup

If PCL visualization headers or libraries are missing in this environment, the `visualization` feature of PCL may not be installed in `vcpkg`. In that case, install the visualization-enabled PCL package before rebuilding.