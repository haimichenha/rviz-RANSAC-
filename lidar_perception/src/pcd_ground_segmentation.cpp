/**
 * @file pcd_ground_segmentation.cpp
 * @brief 实验三 第一问：使用平面拟合法对点云进行地面分割
 * 
 * 功能：
 * 1. 读取 PCD 文件
 * 2. 使用 RANSAC 平面拟合法分割地面点和非地面点
 * 3. 从非地面点中分离出树木（高度 1m-3m）
 * 4. 发布到 ROS 话题，在 Rviz 中可视化
 * 
 * 话题：
 *   /ground_cloud   - 地面点云 (绿色)
 *   /trees_cloud    - 树木点云 (红色)
 *   /other_cloud    - 其他非地面点云 (蓝色)
 * 
 * 用法: rosrun lidar_perception pcd_ground_segmentation <pcd_file>
 */

#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL 基础
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// 滤波器
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

// 地面分割 (RANSAC)
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

// 使用带颜色的点云类型
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

/**
 * @brief 体素下采样
 */
PointCloudXYZ::Ptr voxelGridFilter(const PointCloudXYZ::Ptr& cloud_in, float leaf_size)
{
    PointCloudXYZ::Ptr cloud_out(new PointCloudXYZ);
    pcl::VoxelGrid<PointXYZ> voxel;
    voxel.setInputCloud(cloud_in);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*cloud_out);
    return cloud_out;
}

/**
 * @brief ROI 区域过滤 - 只保留指定范围内的点
 */
PointCloudXYZ::Ptr roiFilter(const PointCloudXYZ::Ptr& cloud_in,
                              float min_x, float max_x,
                              float min_y, float max_y,
                              float min_z, float max_z)
{
    PointCloudXYZ::Ptr cloud_out(new PointCloudXYZ);
    PointCloudXYZ::Ptr temp(new PointCloudXYZ);
    
    pcl::PassThrough<PointXYZ> pass;
    
    // X 方向过滤
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*temp);
    
    // Y 方向过滤
    pass.setInputCloud(temp);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*temp);
    
    // Z 方向过滤
    pass.setInputCloud(temp);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud_out);
    
    return cloud_out;
}

/**
 * @brief RANSAC 平面拟合地面分割
 */
void groundSegmentation(const PointCloudXYZ::Ptr& cloud_in,
                        PointCloudXYZ::Ptr& cloud_ground,
                        PointCloudXYZ::Ptr& cloud_non_ground,
                        pcl::ModelCoefficients::Ptr& coefficients,
                        float distance_threshold = 0.3,
                        int max_iterations = 1000)
{
    cloud_ground.reset(new PointCloudXYZ);
    cloud_non_ground.reset(new PointCloudXYZ);
    coefficients.reset(new pcl::ModelCoefficients);

    if (cloud_in->size() < 10)
    {
        *cloud_non_ground = *cloud_in;
        return;
    }

    // RANSAC 平面分割器
    pcl::SACSegmentation<PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance_threshold);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
        *cloud_non_ground = *cloud_in;
        return;
    }

    // 提取地面点和非地面点
    pcl::ExtractIndices<PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_ground);
    extract.setNegative(true);
    extract.filter(*cloud_non_ground);
}

/**
 * @brief 从非地面点中分离树木 (高度 1m-3m)
 */
void separateTrees(const PointCloudXYZ::Ptr& cloud_non_ground,
                   PointCloudXYZ::Ptr& cloud_trees,
                   PointCloudXYZ::Ptr& cloud_other,
                   float tree_min_height = 0.5,
                   float tree_max_height = 3.5)
{
    cloud_trees.reset(new PointCloudXYZ);
    cloud_other.reset(new PointCloudXYZ);

    for (const auto& pt : cloud_non_ground->points)
    {
        if (pt.z >= tree_min_height && pt.z <= tree_max_height)
            cloud_trees->push_back(pt);
        else
            cloud_other->push_back(pt);
    }
}

/**
 * @brief 将 XYZ 点云转换为带颜色的 XYZRGB 点云
 */
PointCloudT::Ptr colorizeCloud(const PointCloudXYZ::Ptr& cloud_in, uint8_t r, uint8_t g, uint8_t b)
{
    PointCloudT::Ptr cloud_out(new PointCloudT);
    for (const auto& pt : cloud_in->points)
    {
        PointT pt_rgb;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        pt_rgb.r = r;
        pt_rgb.g = g;
        pt_rgb.b = b;
        cloud_out->push_back(pt_rgb);
    }
    return cloud_out;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_ground_segmentation");
    ros::NodeHandle nh;

    std::cout << "============================================" << std::endl;
    std::cout << "  实验三 第一问: 平面拟合法地面分割 (Rviz)" << std::endl;
    std::cout << "============================================" << std::endl;

    // 创建发布者
    ros::Publisher pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1, true);
    ros::Publisher pub_trees = nh.advertise<sensor_msgs::PointCloud2>("/trees_cloud", 1, true);
    ros::Publisher pub_other = nh.advertise<sensor_msgs::PointCloud2>("/other_cloud", 1, true);
    ros::Publisher pub_all = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1, true);

    // 检查参数
    std::string pcd_file;
    if (argc < 2)
    {
        pcd_file = "data/sample.pcd";
        std::cout << "未指定 PCD 文件，使用默认: " << pcd_file << std::endl;
    }
    else
    {
        pcd_file = argv[1];
    }

    // 步骤 1: 读取 PCD 文件
    std::cout << "\n[步骤1] 读取 PCD 文件: " << pcd_file << std::endl;
    PointCloudXYZ::Ptr cloud_raw(new PointCloudXYZ);
    if (pcl::io::loadPCDFile<PointXYZ>(pcd_file, *cloud_raw) == -1)
    {
        std::cerr << "错误: 无法读取文件 " << pcd_file << std::endl;
        return -1;
    }
    std::cout << "原始点云数量: " << cloud_raw->size() << " 个点" << std::endl;

    // 步骤 2: ROI 区域过滤 - 压缩范围，只保留近距离点云
    std::cout << "\n[步骤2] ROI 区域过滤 (压缩范围)..." << std::endl;
    // 只保留 X: -10~10m, Y: -10~10m, Z: -2~5m 范围内的点
    PointCloudXYZ::Ptr cloud_roi = roiFilter(cloud_raw, -10.0f, 10.0f, -10.0f, 10.0f, -2.0f, 5.0f);
    std::cout << "ROI 过滤后点云数量: " << cloud_roi->size() << " 个点" << std::endl;

    // 步骤 3: RANSAC 平面拟合地面分割
    std::cout << "\n[步骤3] RANSAC 平面拟合地面分割..." << std::endl;
    PointCloudXYZ::Ptr cloud_ground, cloud_non_ground;
    pcl::ModelCoefficients::Ptr ground_coefficients;
    groundSegmentation(cloud_roi, cloud_ground, cloud_non_ground, ground_coefficients, 0.3f, 1000);

    std::cout << "平面方程: " 
              << ground_coefficients->values[0] << "x + "
              << ground_coefficients->values[1] << "y + "
              << ground_coefficients->values[2] << "z + "
              << ground_coefficients->values[3] << " = 0" << std::endl;
    std::cout << "地面点数量: " << cloud_ground->size() << std::endl;
    std::cout << "非地面点数量: " << cloud_non_ground->size() << std::endl;

    // 步骤 4: 分离树木
    std::cout << "\n[步骤4] 从非地面点中分离树木 (1m-3m)..." << std::endl;
    PointCloudXYZ::Ptr cloud_trees, cloud_other;
    separateTrees(cloud_non_ground, cloud_trees, cloud_other, 1.0f, 3.0f);
    std::cout << "树木点数量: " << cloud_trees->size() << std::endl;
    std::cout << "其他非地面点数量: " << cloud_other->size() << std::endl;

    // 步骤 5: 着色并合并
    std::cout << "\n[步骤5] 着色点云..." << std::endl;
    PointCloudT::Ptr ground_colored = colorizeCloud(cloud_ground, 0, 255, 0);    // 绿色
    PointCloudT::Ptr trees_colored = colorizeCloud(cloud_trees, 255, 0, 0);      // 红色
    PointCloudT::Ptr other_colored = colorizeCloud(cloud_other, 0, 0, 255);      // 蓝色

    // 合并所有点云
    PointCloudT::Ptr all_colored(new PointCloudT);
    *all_colored = *ground_colored + *trees_colored + *other_colored;

    // 步骤 6: 发布到 ROS 话题
    std::cout << "\n[步骤6] 发布到 ROS 话题..." << std::endl;
    std::cout << "  /ground_cloud    - 地面 (绿色)" << std::endl;
    std::cout << "  /trees_cloud     - 树木 (红色)" << std::endl;
    std::cout << "  /other_cloud     - 其他 (蓝色)" << std::endl;
    std::cout << "  /segmented_cloud - 全部合并" << std::endl;

    sensor_msgs::PointCloud2 msg_ground, msg_trees, msg_other, msg_all;
    pcl::toROSMsg(*ground_colored, msg_ground);
    pcl::toROSMsg(*trees_colored, msg_trees);
    pcl::toROSMsg(*other_colored, msg_other);
    pcl::toROSMsg(*all_colored, msg_all);

    msg_ground.header.frame_id = "map";
    msg_trees.header.frame_id = "map";
    msg_other.header.frame_id = "map";
    msg_all.header.frame_id = "map";

    std::cout << "\n========== 在 Rviz 中查看 ==========" << std::endl;
    std::cout << "1. 打开新终端运行: rviz" << std::endl;
    std::cout << "2. Fixed Frame 设为: map" << std::endl;
    std::cout << "3. Add -> PointCloud2 -> Topic: /segmented_cloud" << std::endl;
    std::cout << "4. 设置 Color Transformer 为 RGB8" << std::endl;
    std::cout << "\n绿色=地面, 红色=树木, 蓝色=其他非地面" << std::endl;
    std::cout << "\n按 Ctrl+C 退出..." << std::endl;

    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        msg_ground.header.stamp = now;
        msg_trees.header.stamp = now;
        msg_other.header.stamp = now;
        msg_all.header.stamp = now;

        pub_ground.publish(msg_ground);
        pub_trees.publish(msg_trees);
        pub_other.publish(msg_other);
        pub_all.publish(msg_all);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
