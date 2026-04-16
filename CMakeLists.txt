cmake_minimum_required(VERSION 3.20)
project(MyCppCode VERSION 0.1.0 LANGUAGES C CXX)

find_package(PCL REQUIRED COMPONENTS common io filters visualization features)

if(NOT TARGET pcl_visualization)
    message(FATAL_ERROR "PCL visualization component is missing. Ensure vcpkg installs pcl[visualization].")
endif()

# 1. Add the executable
add_executable(MyCppCode
    src/main.cpp
    src/PointCloudProcessor.cpp
    src/Visualizer.cpp
)

# 2. Target setup
target_include_directories(MyCppCode PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(MyCppCode PRIVATE ${PCL_DEFINITIONS})
set_property(TARGET MyCppCode PROPERTY CXX_STANDARD 17)

# 3. Link Libraries
target_link_libraries(MyCppCode PRIVATE ${PCL_LIBRARIES})

include(CTest)
enable_testing()