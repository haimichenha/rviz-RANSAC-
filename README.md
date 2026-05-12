# ROS/PCL 激光雷达点云感知与 RANSAC 聚类实践

> 面向简历展示整理的 ROS 点云处理课程/个人实践项目。项目使用 ROS C++ 与 PCL 对激光雷达点云进行预处理、地面分割、障碍物聚类和 RViz 可视化。

## 项目角色与来源

- 项目角色：项目负责人 / 独立完成者
- 项目性质：激光雷达点云处理课程实践与个人整理项目
- 主要工作：ROS C++ 节点编写、PCL 点云预处理、RANSAC 地面分割、欧几里得聚类、障碍物边界框与距离计算、RViz 可视化配置、GitHub README 与项目文档整理。

## 项目简介

本项目实现了一个基础激光雷达点云感知流程：

```text
PointCloud2 输入
  → ROI 过滤
  → VoxelGrid 下采样
  → RANSAC 地面分割
  → 非地面点提取
  → 欧几里得聚类
  → 障碍物边界框/距离计算
  → RViz MarkerArray 可视化
```

项目包含实时点云处理节点，也包含 PCD 文件地面分割和聚类实验程序。

## 技术栈

- Ubuntu 20.04 / ROS Noetic
- C++14 / roscpp
- PCL（Point Cloud Library）
- `sensor_msgs/PointCloud2`
- `pcl_conversions` / `pcl_ros`
- `visualization_msgs/MarkerArray`
- RViz / rosbag / PCD

## 目录结构

```text
.
└── lidar_perception/
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   └── lidar_perception.launch
    ├── rviz/
    │   └── lidar_perception.rviz
    └── src/
        ├── lidar_perception_node.cpp      # 实时点云处理主节点
        ├── pcd_clustering.cpp             # PCD/实时聚类与边界框显示实验
        └── pcd_ground_segmentation.cpp    # PCD 地面分割实验
```

## 功能说明

### 1. 实时点云处理节点

`lidar_perception_node.cpp` 订阅激光雷达点云话题，完成：

- `PassThrough` ROI 区域过滤；
- `VoxelGrid` 体素下采样；
- `SACSegmentation` RANSAC 平面拟合地面；
- `ExtractIndices` 提取地面点和非地面障碍点；
- `EuclideanClusterExtraction` 对障碍物进行聚类；
- 发布下采样点云、地面点云、障碍物点云、聚类点云和 RViz Marker。

### 2. PCD 地面分割实验

`pcd_ground_segmentation.cpp` 用于读取 PCD 文件，使用 RANSAC 平面拟合法分割地面点和非地面点，并将不同类别点云发布到 ROS 话题中供 RViz 查看。

### 3. 聚类与边界框可视化

`pcd_clustering.cpp` 对非地面点进行欧几里得聚类，计算每个聚类的包围盒、中心点和距离，并通过 `MarkerArray` 在 RViz 中显示彩色边界框。

## 编译方式

建议放入 catkin 工作空间：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/haimichenha/rviz-RANSAC-.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行方式

```bash
# 默认订阅 /velodyne_points，并打开 RViz
roslaunch lidar_perception lidar_perception.launch

# 指定输入点云话题
roslaunch lidar_perception lidar_perception.launch input_topic:=/your_points_topic

# 回放 rosbag
roslaunch lidar_perception lidar_perception.launch bag_file:=/path/to/your.bag

# 运行 PCD 地面分割实验
rosrun lidar_perception pcd_ground_segmentation /path/to/your.pcd

# 运行 PCD/点云聚类实验
rosrun lidar_perception pcd_clustering
```

## 主要参数

在 `launch/lidar_perception.launch` 中可调整：

| 参数 | 作用 |
| --- | --- |
| `input_topic` | 输入点云话题，默认 `/velodyne_points` |
| `voxel_leaf_size` | VoxelGrid 下采样体素大小 |
| `ground_threshold` | RANSAC 地面分割距离阈值 |
| `cluster_tolerance` | 欧几里得聚类距离阈值 |
| `min_cluster_size` / `max_cluster_size` | 聚类点数范围 |
| `min_x/max_x/min_y/max_y/min_z/max_z` | ROI 过滤范围 |

## 当前完成度

- 已完成点云预处理、地面分割、障碍物聚类和 RViz 可视化基础流程；
- 已实现实时点云话题处理和 PCD 文件实验程序；
- 项目主要用于课程/个人实践展示，尚未接入完整自动驾驶感知系统。

## 后续优化

- 补充 rosbag/PCD 示例数据或下载说明；
- 增加运行截图和 RViz 效果图；
- 增加参数调优记录和不同场景下的聚类效果对比；
- 增加 CMake/launch 的更严格测试说明；
- 后续可接入目标跟踪、语义分类或导航避障模块。

## 简历描述参考

> ROS/PCL 激光雷达点云感知实践：基于 ROS C++ 和 PCL 实现 PointCloud2 点云处理流程，完成 ROI 过滤、VoxelGrid 下采样、RANSAC 地面分割、欧几里得聚类、障碍物边界框和距离计算，并通过 MarkerArray 在 RViz 中可视化聚类结果。
