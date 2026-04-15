cmake_minimum_required(VERSION 3.20)
project(MyCppCode VERSION 0.1.0 LANGUAGES C CXX)

# 1. Find packages. 
# It's best practice to list the components you need explicitly.
find_package(fmt CONFIG REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters visualization)

# 2. Add the executable
add_executable(MyCppCode VoxelGridFilter.cpp)

# 3. PCL Setup (Important for Windows)
# PCL often doesn't "bundle" headers into the target automatically, 
# so we manually add the include directories and preprocessor definitions.
target_include_directories(MyCppCode PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(MyCppCode PRIVATE ${PCL_DEFINITIONS})
set_property(TARGET MyCppCode PROPERTY CXX_STANDARD 17)

# 4. Link Libraries
# Based on your log, the targets are named pcl_common, pcl_io, etc.
target_link_libraries(MyCppCode PRIVATE 
    fmt::fmt 
    pcl_common 
    pcl_io 
    pcl_filters
    pcl_visualization
)

include(CTest)
enable_testing()