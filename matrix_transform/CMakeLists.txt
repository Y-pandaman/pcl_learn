cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl-matrix_transform)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})

# PCL_INCLUDE_DIRS：设置为PCL安装的标头和依赖标头的路径
# PCL_LIBRARY_DIRS：设置为PCL库和第3方依赖关系所在的路径
# PCL_DEFINITIONS：列出所需的预处理程序定义和编译器标志
# PCL_LIBRARIES：设置为已建立和已安装的PCL库的文件名