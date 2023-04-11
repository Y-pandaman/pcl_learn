cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(GujiyizuBiaomianfaxian)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}) # 设置为PCL安装的标头和依赖标头的路径
link_directories(${PCL_LIBRARY_DIRS})  # 设置为PCL库和第3方依赖关系所在的路径
add_definitions(${PCL_DEFINITIONS})  # 列出所需的预处理程序定义和编译器标志

add_executable (GujiyizuBiaomianfaxian GujiyizuBiaomianfaxian.cpp)
target_link_libraries (GujiyizuBiaomianfaxian ${PCL_LIBRARIES})  # 设置为已建立和已安装的PCL库的文件名