cmake_minimum_required(VERSION 3.1)
project(transform_demo)

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} transform_demo.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
