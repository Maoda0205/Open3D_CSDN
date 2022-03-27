cmake_minimum_required(VERSION 2.8)
project(open3d_test)

set(CMAKE_CXX_STANDARD 11)

find_package( Open3D  REQUIRED)
include_directories("/usr/local/include/open3d/")
link_directories(${Open3D_LIBRARY_DIRS})


add_executable(Point_cloud Kd_tree.cpp)
target_link_libraries(Point_cloud ${Open3D_LIBRARIES})

target_include_directories(Point_cloud PUBLIC ${Open3D_INCLUDE_DIRS})
