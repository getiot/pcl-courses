cmake_minimum_required(VERSION 3.0)
project(pcd_example)

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(save_pcd_ascii save_pcd_ascii.cpp)
add_executable(save_pcd_binary save_pcd_binary.cpp)
add_executable(pcd_viewer pcd_viewer.cpp)

target_link_libraries(save_pcd_ascii ${PCL_LIBRARIES})
target_link_libraries(save_pcd_binary ${PCL_LIBRARIES})
target_link_libraries(pcd_viewer ${PCL_LIBRARIES} ${VTK_LIBRARIES})

install(TARGETS pcd_viewer RUNTIME DESTINATION bin)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY
)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)