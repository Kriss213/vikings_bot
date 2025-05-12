cmake_minimum_required(VERSION 3.8)
project(courier_obstacle_plugin)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_ros REQUIRED)

set(dep_pkgs
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    nav2_costmap_2d
    pluginlib
    pcl_ros)

# === Build ===

add_library(${lib_name} SHARED
            src/courier_obstacle_layer.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs courier_obstacle_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d courier_obstacle_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
