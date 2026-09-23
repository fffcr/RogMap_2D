cmake_minimum_required(VERSION 3.14.4)
project(rog_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)
add_compile_options(-Wno-dev)
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0054 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress developer warnings")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DUSE_ROS2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)      
find_package(tf2_ros REQUIRED)              

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)
add_library(rog_map STATIC ${srcs})

target_include_directories(rog_map PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        ${pcl_conversions_INCLUDE_DIRS} 
)

target_link_libraries(rog_map PUBLIC
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${tf2_ros_TARGETS}                   
        ${message_filters_TARGETS}
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        yaml-cpp
        -ldw
)

#=================独立调试节点============================#
# 入口放 Apps/ 而不是 src/，这样不会被上面 GLOB_RECURSE src/*.cpp 收进静态库
# （否则库里也有一份 main，链接时行为依赖链接顺序，不干净）。
add_executable(rog_map_node
        Apps/rog_map_node.cpp
)

target_link_libraries(rog_map_node PUBLIC
        rog_map
)

ament_target_dependencies(rog_map_node PUBLIC
        rclcpp
)

#=================INSTALL============================#
install(TARGETS rog_map
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS rog_map_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

# install the launch and config files
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()