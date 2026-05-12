cmake_minimum_required(VERSION 3.0.2)
project(lidar_perception)

## Compile as C++14, required for modern PCL
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
)

## Find PCL
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs visualization_msgs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable - 主节点：地面分割与聚类
add_executable(lidar_perception_node src/lidar_perception_node.cpp)

## Add cmake target dependencies
add_dependencies(lidar_perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link
target_link_libraries(lidar_perception_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## PCD 文件地面分割程序 (实验三: 平面拟合法)
add_executable(pcd_ground_segmentation src/pcd_ground_segmentation.cpp)
add_dependencies(pcd_ground_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_ground_segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## PCD 文件聚类程序 (实验三: 聚类与边框)
add_executable(pcd_clustering src/pcd_clustering.cpp)
add_dependencies(pcd_clustering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_clustering
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
