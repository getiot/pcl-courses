cmake_minimum_required(VERSION 3.0)
project(pcl_hello LANGUAGES CXX)

# 设置 PCL 库的路径
find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(${PROJECT_NAME} main.cpp)

# 链接 PCL 库
target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${VTK_LIBRARIES})
